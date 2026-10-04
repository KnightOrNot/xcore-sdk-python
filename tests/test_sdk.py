import pytest

from xcoresdk_python import XCoreError
from xcoresdk_python.sdk import load_sdk, sdk_directory


def test_missing_binary_has_actionable_message(tmp_path):
    with pytest.raises(XCoreError, match="matching Python.*--sdk-dir"):
        load_sdk(str(tmp_path))


def test_sdk_location_can_be_configured(monkeypatch, tmp_path):
    monkeypatch.setenv("XCORE_SDK_DIR", str(tmp_path))
    assert sdk_directory() == tmp_path


def test_incompatible_extension_is_not_loaded(tmp_path):
    (tmp_path / "xCoreSDK_python.cpython-310-x86_64-linux-gnu.so").touch()
    with pytest.raises(XCoreError, match="cpython-310"):
        load_sdk(str(tmp_path))
