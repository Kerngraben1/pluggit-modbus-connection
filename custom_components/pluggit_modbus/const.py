"""The constants."""

# ============================================================================
# Vibe Code Änderung (2026-11-24): Domain "pluggit" -> "pluggit_modbus"
# (Ordnername, manifest.json und services.yaml wurden entsprechend umbenannt)
# ============================================================================
DOMAIN = "pluggit_modbus"  # Vibe Code: geändert von "pluggit"
DEFAULT_NAME = "Pluggit"
DEFAULT_SCAN_INTERVAL = 30  # Updated default for new installations
DEFAULT_PORT = 502

DEVICE_TYPES = {
    1: "AP190",
    2: "AP310",
    3: "AP460",
    4: "AD160"
}
