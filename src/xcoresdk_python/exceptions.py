"""Errors with enough context to diagnose SDK and robot failures."""


class XCoreError(RuntimeError):
    """SDK loading, communication, or robot operation failed."""
