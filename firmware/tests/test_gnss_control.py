# SPDX-License-Identifier: MIT
import json
from pathlib import Path
import re
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import gnss_control as control


class CatalogTests(unittest.TestCase):
    def test_complete_audit_scope(self):
        catalog = control.CATALOG
        self.assertEqual(len(catalog["inventory"]), 81)
        expected = {row["name"] for row in catalog["inventory"] if row["policy"] in "RD"}
        self.assertEqual(expected, set(catalog["settings"]))
        self.assertEqual(len(expected), 36)
        source = (ROOT / "core/src/gnss_control.c").read_text()
        firmware = set(re.findall(r'\{"(\w+)",\s*\d,\s*\d\}', source))
        self.assertEqual(expected, firmware)

    def test_every_catalog_setting_builds(self):
        for name, spec in control.CATALOG["settings"].items():
            assignments = []
            for field in spec["fields"]:
                if not field.get("writable", True):
                    continue
                if "enum" in field:
                    value = field["enum"][0]
                elif "items" in field:
                    value = field["items"][0]
                elif "min" in field:
                    value = str(field["min"])
                elif field.get("satellite") or field.get("satellite_list"):
                    value = "G01"
                elif "pattern" in field:
                    value = "Ch01"
                else:
                    value = "A"
                assignments.append(field["name"] + "=" + value)
            with self.subTest(name=name):
                command = control.usb_set(name, assignments)
                self.assertTrue(command.startswith("gnss write set" + name + ","))
                self.assertLessEqual(len(command), control.MAX_USB)

    def test_typed_independent_dynamics(self):
        self.assertEqual(
            control.usb_set("ReceiverDynamics", ["Motion=Static"]),
            "gnss write setReceiverDynamics,,Static",
        )
        self.assertEqual(
            control.usb_set("ReceiverDynamics", ["Level=Low"]), "gnss write setReceiverDynamics,Low"
        )
        self.assertEqual(
            control.CATALOG["startup_defaults"]["ReceiverDynamics"],
            {"Level": "Low", "Motion": "Static"},
        )
        self.assertEqual(
            control.usb_set("AntennaOffset", ["Antenna=Main", "Type=A B"]),
            'gnss write setAntennaOffset,Main,,,,"A B"',
        )

    def test_units_bounds_and_scope(self):
        invalid = [
            ("CN0Mask", ["Signal=GPSL2PY", "Mask=10"]),
            ("CN0Mask", ["Signal=GALE6BC", "Mask=61"]),
            ("CN0Mask", ["Signal=GALE6BC", "Mask=nan"]),
            ("CN0Mask", ["Signal=GALE6BC", "Mask=10.5"]),
            ("PVTMode", ["RoverMode=StandAlone+RTK"]),
            ("PVTMode", ["RoverMode=all"]),
            ("DiffCorrMaxAge", ["RTKCorr=10"]),
            ("DiffCorrUsage", ["BaseSelection=manual"]),
            ("GalOSNMAUsage", ["MTRoot=key"]),
            ("PPSParameters", ["Interval=MHz10"]),
            ("StaticPosGeodetic", ["Position=Geodetic1", "Latitude=91"]),
            ("AntennaOffset", ["Antenna=Aux1", "DeltaU=1"]),
            ("SignalTracking", ["Signal=GLOL2P"]),
            ("SignalTracking", ["Signal=GPSL1CA+GPSL1CA"]),
            ("COMSettings", ["BaudRate=baud9600"]),
            ("MarkerParameters", ["MarkerName=A\nfactoryReset"]),
        ]
        for name, values in invalid:
            with self.subTest(name=name, values=values), self.assertRaises(ValueError):
                control.usb_set(name, values)
        self.assertEqual(
            control.usb_set("DiffCorrMaxAge", ["PPPCorr=3600"]),
            "gnss write setDiffCorrMaxAge,,,3600",
        )
        self.assertEqual(
            control.usb_set("DiffCorrUsage", ["MaxAge=100"]), "gnss write setDiffCorrUsage,,100"
        )
        self.assertEqual(
            control.usb_set("SignalUsage", ["NavData=+GALE6BC"]),
            "gnss write setSignalUsage,,+GALE6BC",
        )

    def test_signal_and_satellite_list_edits(self):
        for name, assignment, expected in [
            ("SignalTracking", "Signal=all", "all"),
            ("SignalTracking", "Signal=GALILEO", "GALILEO"),
            ("SignalTracking", "Signal=+GPSL2PY", "+GPSL2PY"),
            ("SatelliteTracking", "Satellite=+G38+G39+S158+I14", "+G38+G39+S158+I14"),
            ("SatelliteUsage", "Satellite=-GALILEO", "-GALILEO"),
        ]:
            with self.subTest(name=name, assignment=assignment):
                self.assertEqual(
                    control.usb_set(name, [assignment]), f"gnss write set{name},{expected}"
                )
        for name, assignment in [
            ("SignalUsage", "PVT=GEOL1"),
            ("SignalUsage", "PVT=GEOL5"),
            ("SignalTracking", "Signal=all+GPSL1CA"),
            ("SatelliteTracking", "Satellite=G37"),
            ("SatelliteTracking", "Satellite=I15"),
            ("SatelliteUsage", "Satellite=G33"),
            ("SatelliteUsage", "Satellite=SBAS"),
            ("SatelliteUsage", "Satellite=I14"),
        ]:
            with self.subTest(name=name, assignment=assignment), self.assertRaises(ValueError):
                control.usb_set(name, [assignment])

    def test_loss_aware_reply_chunks(self):
        raw = b"$R: getReceiverDynamics\r\nReceiverDynamics,Low,Static\r\nCOM1>"
        lines = [
            b"type=gnss_control v=1 seq=1 stage=before status=started ascii_end=0\r\n",
            f"type=gnss_control v=1 seq=1 stage=done status=complete ascii_end={len(raw)} drops=0 rx_loss=0 command_hex={b'getReceiverDynamics'.hex().upper()}\r\n".encode(),
            b"type=gnss_ascii v=1 offset=0 hex=" + raw.hex().upper().encode() + b"\r\n",
        ]

        class Peer:
            in_waiting = 1

            def write(self, data):
                self.sent = data

            def read(self, size):
                return lines.pop(0) if lines else b""

        peer = Peer()
        result = control.exchange(peer, "gnss read getReceiverDynamics", timeout=5)
        self.assertTrue(result["complete"])
        self.assertIn("Low,Static", result["receiver_text"])
        self.assertEqual(peer.sent, b"gnss read getReceiverDynamics\r\n")

    def test_provisioning_redaction(self):
        output = control.sanitize("setUserAccessLevel,admin,password\nReceiverDynamics,Low,Static")
        self.assertNotIn("password", output)
        self.assertIn("Low,Static", output)

    def test_exchange_matches_operation_and_fails_busy_without_ascii(self):
        class Peer:
            in_waiting = 1

            def __init__(self, lines):
                self.lines = lines

            def write(self, data):
                self.sent = data

            def read(self, size):
                return self.lines.pop(0) if self.lines else b""

        raw = b"ReceiverDynamics,Low,Static\r\n"
        stale = b"type=gnss_control v=1 seq=6 stage=before ascii_end=0\n"
        stale += (
            b"type=gnss_control v=1 seq=6 stage=done status=complete ascii_end=0 "
            + b"command_hex="
            + b"getPVTMode".hex().encode()
            + b"\n"
        )
        lines = [
            stale,
            b"type=gnss_control v=1 seq=7 stage=before ascii_end=100\n",
            f"type=gnss_control v=1 seq=7 stage=done status=complete ascii_end={100+len(raw)} command_hex={b'getReceiverDynamics'.hex()}\n".encode(),
            b"type=gnss_ascii v=1 offset=100 hex=" + raw.hex().encode() + b"\n",
        ]
        result = control.exchange(Peer(lines), "gnss read getReceiverDynamics", timeout=5)
        self.assertTrue(result["complete"])
        self.assertEqual(result["receiver_text"], raw.decode().strip())
        # Busy can refer to the previous operation's absolute ASCII position.
        # It must fail promptly, without waiting for that older reply's bytes.
        with self.assertRaisesRegex(RuntimeError, "Firmware busy"):
            control.exchange(
                Peer([b"type=gnss_control v=1 seq=7 stage=denied status=busy ascii_end=9999\n"]),
                "gnss read getReceiverDynamics",
                timeout=5,
            )
        with self.assertRaisesRegex(RuntimeError, "Firmware rejected command: invalid"):
            control.exchange(
                Peer(
                    [
                        b"type=gnss_control v=1 seq=7 kind=0 stage=done status=invalid ascii_end=9999 command_hex=\n"
                    ]
                ),
                "gnss read getUnsupported",
                timeout=5,
            )
        with self.assertRaisesRegex(RuntimeError, "Firmware rejected USB command"):
            control.exchange(
                Peer(
                    [
                        b"type=hybrid_command accepted=0 command_hex="
                        + b"system reboot mcu".hex().upper().encode()
                        + b"\n"
                    ]
                ),
                "system reboot mcu",
                timeout=5,
            )
        defaults = control.exchange(
            Peer(
                [
                    stale,
                    b"type=gnss_control v=1 seq=8 stage=done status=defaults_saved saved=1 ascii_end=0 command_hex=\n",
                ]
            ),
            "system defaults",
            timeout=5,
        )
        self.assertTrue(defaults["complete"])
        reboot = control.exchange(
            Peer(
                [
                    b"type=system_reboot v=1 target=gnss stage=accepted\n",
                    b"type=system_reboot v=1 target=mcu stage=accepted\n",
                ]
            ),
            "system reboot mcu",
            timeout=5,
        )
        self.assertEqual(reboot["events"][-1]["target"], "mcu")
        self.assertFalse(reboot["physical_reboot_verified"])

    def test_exchange_requires_before_and_complete_chunk_range(self):
        class Peer:
            in_waiting = 1

            def __init__(self, lines):
                self.lines = lines

            def write(self, data):
                pass

            def read(self, size):
                return self.lines.pop(0) if self.lines else b""

        done = (
            f"type=gnss_control v=1 seq=1 stage=done status=complete ascii_end=2 "
            f"command_hex={b'getPVTMode'.hex()}\n"
        ).encode()
        for before, chunk in [
            (b"", b"offset=0 hex=6162"),
            (b"type=gnss_control v=1 seq=1 stage=before ascii_end=0\n", b"offset=1 hex=62"),
        ]:
            with self.subTest(before=before, chunk=chunk), self.assertRaises(RuntimeError):
                control.exchange(
                    Peer([before + done, b"type=gnss_ascii v=1 " + chunk + b"\n"]),
                    "gnss read getPVTMode",
                    timeout=0.1,
                )

    def test_catalog_matches_scalar_and_table_mib_argument_definitions(self):
        scalar = """-- CalibCommonDelay ARGUMENTS

ccoCommonDelay  OBJECT-TYPE
    SYNTAX Integer32 (-10000000..10000000)
  ::= { cCalibCommonDelay 1 }
-- ========================================
"""
        self.assertIn(
            "ccocommondelay",
            control.require_installed_catalog("CalibCommonDelay", "mosaic-G5 T 1.1.0", scalar),
        )
        table = """-- CalibSignalDelay ARGUMENTS

csiSignal OBJECT-TYPE
    SYNTAX Integer32 { GPSL1CA(1) }
  ::= { cCalibSignalDelay 1 }
csiSignalDelay  OBJECT-TYPE
    SYNTAX Integer32 (-1000000..1000000)
  ::= { cCalibSignalDelay 2 }
-- ========================================
"""
        control.require_installed_catalog("CalibSignalDelay", "mosaic-G5 T 1.1.0", table)

    def test_catalog_rejects_missing_renamed_wrong_family_and_spoofed_fields(self):
        with self.assertRaisesRegex(ValueError, "lacks ccoCommonDelay"):
            control.require_installed_catalog(
                "CalibCommonDelay",
                "mosaic-G5 T 1.1.0",
                "-- CalibCommonDelay ARGUMENTS\nccoCommonDelayExtra OBJECT-TYPE\n SYNTAX Integer32 (0..1)\n ::= { c 1 }\n",
            )
        with self.assertRaisesRegex(ValueError, "lacks csiSignalDelay"):
            control.require_installed_catalog(
                "CalibSignalDelay",
                "mosaic-G5 T 1.1.0",
                "-- CalibSignalDelay ARGUMENTS\ncsiSignal OBJECT-TYPE\n SYNTAX Integer32 { GPSL1CA(1) }\n ::= { c 1 }\ncsiRenamed OBJECT-TYPE\n SYNTAX Integer32 (0..1)\n ::= { c 2 }\n",
            )
        with self.assertRaisesRegex(ValueError, "lacks the CalibCommonDelay ARGUMENTS"):
            control.require_installed_catalog(
                "CalibCommonDelay",
                "mosaic-G5 T 1.1.0",
                "-- OtherFamily ARGUMENTS\nccoCommonDelay OBJECT-TYPE\n SYNTAX Integer32 (0..1)\n ::= { c 1 }\n",
            )
        with self.assertRaisesRegex(ValueError, "lacks ccoCommonDelay"):
            control.require_installed_catalog(
                "CalibCommonDelay",
                "mosaic-G5 T 1.1.0",
                '-- CalibCommonDelay ARGUMENTS\nDESCRIPTION "ccoCommonDelay OBJECT-TYPE"\n',
            )

    def test_catalog_keeps_identity_gate_and_unmapped_family_closed(self):
        with self.assertRaises(ValueError):
            control.require_installed_catalog(
                "CalibCommonDelay",
                "mosaic-G5 T 2.0.0",
                "-- CalibCommonDelay ARGUMENTS\nccoCommonDelay OBJECT-TYPE\n",
            )
        with self.assertRaises(ValueError):
            control.require_installed_catalog(
                "DiffCorrUsage",
                "mosaic-G5 T 1.1.0",
                "-- DiffCorrUsage ARGUMENTS\n-- ========================================\n",
            )

    def test_installed_enum_blocks_catalog_value_missing_from_receiver(self):
        mib = """-- SignalAuthentication ARGUMENTS
naLevel OBJECT-TYPE
    SYNTAX Integer32 {
        Detection(1) }
    UnitParts " , , Level, "
  ::= { cSignalAuthentication 1 }
-- ========================================
"""
        args = control.require_installed_catalog("SignalAuthentication", "mosaic-G5 T 1.1.0", mib)
        control.validate_installed_enum(
            "SignalAuthentication", "gnss write setSignalAuthentication,Detection", args
        )
        with self.assertRaisesRegex(ValueError, "allowed: detection"):
            control.validate_installed_enum(
                "SignalAuthentication", "gnss write setSignalAuthentication,Mitigation", args
            )

    def test_fitted_signal_removal_limit_keeps_satellite_edits_and_signal_addition(self):
        for name, field in [("SignalTracking", "Signal"), ("SignalUsage", "NavData")]:
            with (
                self.subTest(name=name),
                self.assertRaisesRegex(ValueError, "complete replacement"),
            ):
                control.usb_set(name, [field + "=-GALE6BC"])
            self.assertIn("+GALE6BC", control.usb_set(name, [field + "=+GALE6BC"]))
        self.assertIn("-E36", control.usb_set("SatelliteTracking", ["Satellite=-E36"]))
        self.assertIn("-E36", control.usb_set("SatelliteUsage", ["Satellite=-E36"]))

    def test_verified_catalog_symbols_and_recorded_pvt_family(self):
        for name, setting in control.CATALOG["settings"].items():
            if name != "DiffCorrUsage":
                self.assertTrue(all(field.get("mib_name") for field in setting["fields"]), name)
        # Symbols read from a production board, G5-T 1.1.0, 2026-10-22.
        mib = """-- PVTMode ARGUMENTS
pmMode OBJECT-TYPE
    SYNTAX Integer32 { Static(1), Rover(2) }
  ::= { cPVTMode 1 }
pmRoverMode OBJECT-TYPE
    SYNTAX BITS { StandAlone(0), SBAS(1), PPP(5) }
  ::= { cPVTMode 2 }
pmRefPos OBJECT-TYPE
    SYNTAX Integer32 { auto(-2), Geodetic1(1) }
  ::= { cPVTMode 3 }
-- ========================================
"""
        control.require_installed_catalog("PVTMode", "mosaic-G5 T 1.1.0", mib)


if __name__ == "__main__":
    unittest.main()
