#!/usr/bin/env python3
"""Contract checks for the operator-facing runtime guide."""

from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
GUIDE = REPOSITORY_ROOT / "docs" / "RUNTIME_USER_GUIDE.md"
README = REPOSITORY_ROOT / "README.md"


class RuntimeUserGuideTests(unittest.TestCase):
    def test_guide_documents_real_operator_interfaces(self) -> None:
        text = GUIDE.read_text(encoding="utf-8")

        required = (
            "./scripts/run_fastlio_jt128_pc.sh",
            "./scripts/run_a2w_mapping.sh",
            "./scripts/run_a2w_localization.sh maps/factory_map",
            "/mapping/save_map_bundle",
            "a2w_fastlio_msgs/srv/SaveMapBundle",
            "ros2 run a2w_fastlio_map map_bundle_inspect maps/factory_map",
            "/mapping/registration_status",
            "/localization/registration_status",
            "/localization/status",
            "./scripts/run_jt128_full_validation.sh",
            "map → camera_init → body",
            "INITIALIZING",
            "RELOCALIZING",
            "当前未实现",
            "Offline implementation and verification complete; JT128 hardware validation pending.",
        )
        for value in required:
            with self.subTest(value=value):
                self.assertIn(value, text)

    def test_guide_does_not_claim_unsupported_initialization_or_manual_service(self) -> None:
        text = GUIDE.read_text(encoding="utf-8")

        self.assertIn("当前冷启动使用 GlobalRelocalizer", text)
        self.assertIn("initialization.mode=fixed_region", text)
        self.assertIn("手动重定位 Service：当前未实现", text)
        self.assertIn("Mapping 与 Localization 不能同时运行", text)

    def test_readme_links_to_runtime_guide(self) -> None:
        text = README.read_text(encoding="utf-8")
        self.assertIn("[完整运行与使用手册](docs/RUNTIME_USER_GUIDE.md)", text)


if __name__ == "__main__":
    unittest.main()
