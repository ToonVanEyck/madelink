# Import common functions from main decoder (will be injected at runtime)
# PROPERTY_IDS = {}
# PAYLOAD_DECODERS = {}
# configure_property = None  # Will be set by main decoder

def setup_properties(configure_property_func):
    """Setup function called by main decoder to configure properties"""
    global PROPERTY_IDS, PAYLOAD_DECODERS, configure_property
    PROPERTY_IDS = {}
    PAYLOAD_DECODERS = {}
    configure_property = lambda pid, name, dec=None: configure_property_func(PROPERTY_IDS, PAYLOAD_DECODERS, pid, name, dec)

def configure_default_properties():
    """Configure all properties with generic names (no custom decoders)"""
    for i in range(64):
        configure_property(i, f"property_{i:02d}")

