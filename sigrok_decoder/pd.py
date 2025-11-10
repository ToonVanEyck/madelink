import sigrokdecode as srd
from common.srdhelper import bitpack
from math import floor, ceil
import os
import importlib.util

# Common property configuration system
def default_decoder(property_id, data_bytes, checksum_byte, property_ids=None):
    """Default decoder - just hex representation of COBS decoded payload"""
    if property_ids is None:
        property_ids = {}
    hex_str = ' '.join(['%02X' % b for b in data_bytes])
    ascii_str = ''.join([chr(b) if 32 <= b <= 126 else '.' for b in data_bytes])
    property_name = property_ids.get(property_id, f'property_0x{property_id:02X}')
    
    # Show the raw hex data in main display levels
    if hex_str:
        display_str = hex_str  # Just the clean hex data
    else:
        display_str = "(no data)"
    
    return {
        'decoded': display_str,
        'ascii': ascii_str,
        'info': f'{property_name}: {hex_str}'  # Just show property name in overview level
    }

def configure_property(property_ids_dict, payload_decoders_dict, property_id, property_name, decoder=None):
    """
    Configure a property with its name and optional custom decoder
    
    Args:
        property_ids_dict: Dictionary to store property ID mappings
        payload_decoders_dict: Dictionary to store custom decoders
        property_id: Property ID (0-63)
        property_name: Human-readable name for the property
        decoder: Optional custom decoder function
    """
    if not (0 <= property_id <= 63):
        raise ValueError(f"Property ID must be 0-63, got {property_id}")
    
    property_ids_dict[property_id] = property_name
    
    if decoder is not None:
        payload_decoders_dict[property_id] = decoder

def create_decode_payload_function(property_ids_dict, payload_decoders_dict):
    """Create a decode_payload function that uses the configured decoders"""
    def decode_payload(property_id, data_bytes, checksum_byte):
        # Use custom decoder if configured, otherwise use default
        if property_id in payload_decoders_dict:
            decoder_func = payload_decoders_dict[property_id]
            return decoder_func(data_bytes, checksum_byte)
        else:
            return default_decoder(property_id, data_bytes, checksum_byte, property_ids_dict)
    return decode_payload

# Dynamic loading of property definitions and payload decoders
def get_available_sub_decoders():
    """Get list of available sub-decoder files"""
    try:
        decoder_dir = os.path.join(os.path.dirname(__file__), 'sub_decoders')
        if os.path.exists(decoder_dir):
            files = [f[:-3] for f in os.listdir(decoder_dir) 
                    if f.endswith('.py') and f != '__init__.py']
            return files if files else ['default']
        return ['default']
    except:
        return ['default']

def load_sub_decoder(decoder_name):
    """Load and initialize a sub-decoder module"""
    try:
        decoder_path = os.path.join(os.path.dirname(__file__), 'sub_decoders', f'{decoder_name}.py')
        if os.path.exists(decoder_path):
            spec = importlib.util.spec_from_file_location(f"sub_decoders.{decoder_name}", decoder_path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        else:
            # Fallback to default
            decoder_path = os.path.join(os.path.dirname(__file__), 'sub_decoders', 'default.py')
            spec = importlib.util.spec_from_file_location("sub_decoders.default", decoder_path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            decoder_name = 'default'

        # Initialize the module with common functions
        if hasattr(module, 'setup_properties'):
            module.setup_properties(configure_property)
            
            # Call the appropriate configuration function
            config_func_name = f'configure_{decoder_name}_properties'
            if hasattr(module, config_func_name):
                getattr(module, config_func_name)()
            
            # Create the decode_payload function
            if hasattr(module, 'PROPERTY_IDS') and hasattr(module, 'PAYLOAD_DECODERS'):
                module.decode_payload = create_decode_payload_function(module.PROPERTY_IDS, module.PAYLOAD_DECODERS)
            
        return module
            
    except Exception as e:
        # Create minimal fallback
        class FallbackModule:
            def __init__(self):
                self.PROPERTY_IDS = {i: f"property_{i:02d}" for i in range(64)}
                self.PAYLOAD_DECODERS = {}
                self.decode_payload = create_decode_payload_function(self.PROPERTY_IDS, self.PAYLOAD_DECODERS)
        return FallbackModule()

# Action type definitions
ACTION_TYPES = {
    0: "READ",
    1: "WRITE", 
    2: "BROADCAST",
    3: "SYNC"
}

# Given a parity type to check (odd, even, zero, one), the value of the
# parity bit, the value of the data, and the length of the data (5-9 bits,
# usually 8 bits) return True if the parity is correct, False otherwise.
# 'none' is _not_ allowed as value for 'parity_type'.
def parity_ok(parity_type, parity_bit, data, num_data_bits):

    # Handle easy cases first (parity bit is always 1 or 0).
    if parity_type == 'zero':
        return parity_bit == 0
    elif parity_type == 'one':
        return parity_bit == 1

    # Count number of 1 (high) bits in the data (and the parity bit itself!).
    ones = bin(data).count('1') + parity_bit

    # Check for odd/even parity.
    if parity_type == 'odd':
        return (ones % 2) == 1
    elif parity_type == 'even':
        return (ones % 2) == 0

def cobs_decode(data):
    """
    Decode COBS (Consistent Overhead Byte Stuffing) encoded data.
    Returns decoded bytes and success flag.
    """
    if not data:
        return [], False
    
    decoded = []
    i = 0
    
    while i < len(data):
        code = data[i]
        
        if code == 0:
            # Zero delimiter marks end of packet
            break
            
        if code > len(data) - i:
            # Invalid code - points beyond available data
            return [], False
            
        # Copy the next (code-1) bytes
        for j in range(1, code):
            if i + j >= len(data):
                return [], False
            decoded.append(data[i + j])
        
        # Add a zero if code < 255 (unless it's the last segment)
        if code < 255 and i + code < len(data) and data[i + code] != 0:
            decoded.append(0)
            
        i += code
    
    return decoded, True

def parse_madelink_header(header_bytes, property_ids=None):
    """
    Parse the 3-byte Madelink header according to specification.
    Returns a dictionary with parsed fields and validation info.
    """
    if len(header_bytes) != 3:
        return None
    
    if property_ids is None:
        property_ids = {}
    
    # Convert 3 bytes to 24-bit value (big-endian)
    header_value = (header_bytes[0] << 16) | (header_bytes[1] << 8) | header_bytes[2]
    
    # Extract fields based on actual bit layout in header bytes
    # Byte 0: [action1:action0] [staging] [prop_msb4:prop_msb0]
    # Byte 1: [prop_lsb] [node_cnt_bits 6:0] 
    # Byte 2: [node_cnt_bits 12:7] [parity]
    
    action = (header_bytes[0] >> 6) & 0x03         # bits 7-6 of byte 0
    staging_error = (header_bytes[0] >> 5) & 0x01  # bit 5 of byte 0
    
    # Extract property_id split across bytes as per C implementation:
    # 5 MSB bits in byte 0 (bits 4-0), 1 LSB bit in byte 1 (bit 7)
    prop_msb = header_bytes[0] & 0x1F              # 5 MSB bits from byte 0, bits 4-0  
    prop_lsb = (header_bytes[1] >> 7) & 0x01       # 1 LSB bit from byte 1, bit 7
    property_id = (prop_msb << 1) | prop_lsb       # Combine: MSB in upper bits, LSB in lower bit
    
    # Extract node_cnt exactly matching the C implementation:
    # 6 LSB bits from byte 1 (bits 5-0), 7 MSB bits from byte 2 (bits 7-1)
    node_cnt_lsb = header_bytes[1] & 0x3F           # 6 LSB bits from byte 1, bits 5-0
    node_cnt_msb = (header_bytes[2] >> 1) & 0x7F    # 7 MSB bits from byte 2, bits 7-1  
    node_cnt = (node_cnt_msb << 6) | node_cnt_lsb   # MSB in upper bits, LSB in lower bits
    
    parity_bit = header_value & 0x01              # bit 0
    
    # Calculate even parity for validation
    parity_calc = 0
    temp_val = header_value >> 1  # Exclude the parity bit itself
    while temp_val:
        parity_calc ^= (temp_val & 1)
        temp_val >>= 1
    
    parity_valid = (parity_calc == parity_bit)
    
    # Get human-readable strings
    action_str = ACTION_TYPES.get(action, f"Unknown({action})")
    property_str = property_ids.get(property_id, f"Unknown(0x{property_id:02X})")
    
    # Determine staging/error bit meaning based on action
    if action in [1, 2]:  # WRITE or BROADCAST
        staging_str = "Staged" if staging_error else "Commit"
    elif action == 0:     # READ
        staging_str = ""
        property_str = "node error" if staging_error else property_str
    else:                 # SYNC or unknown
        staging_str = f"Bit21={staging_error}"
    
    return {
        'raw_value': header_value,
        'action': action,
        'action_str': action_str,
        'staging_error': staging_error,
        'staging_error_str': staging_str,
        'property_id': property_id,
        'property_str': property_str,
        'node_cnt': node_cnt,
        'parity_bit': parity_bit,
        'parity_valid': parity_valid,
        'parity_calc': parity_calc
    }

def parse_sync_header(header_byte):
    """
    Parse a 1-byte SYNC header.
    Bits 7-6: Action (should be 3 for SYNC)
    Bits 5-4: Sync type (0=Acknowledge, 1=Commit, 2-3=Reserved)
    Bits 3-0: Error code (bit flags: 3=Other, 2=Transmission, 1=Property Handler, 0=Property Not Supported)
    """
    # Extract action from bits 7-6 (should be 3 for SYNC)
    action = (header_byte >> 6) & 0x03
    
    # Extract sync type from bits 5-4
    sync_type = (header_byte >> 4) & 0x03
    sync_type_names = {
        0: "Acknowledge",
        1: "Commit", 
        2: "Reserved",
        3: "Reserved"
    }
    sync_type_str = sync_type_names.get(sync_type, f"Unknown({sync_type})")
    
    # Extract error code from bits 3-0
    error_code = header_byte & 0x0F
    error_flags = []
    if error_code & 0x08:  # bit 3
        error_flags.append("Other Error")
    if error_code & 0x04:  # bit 2
        error_flags.append("Transmission Error")
    if error_code & 0x02:  # bit 1
        error_flags.append("Property Handler Error")
    if error_code & 0x01:  # bit 0
        error_flags.append("Property Not Supported")
    
    error_str = ", ".join(error_flags) if error_flags else "No Error"
    
    return {
        'raw_value': header_byte,
        'action': action,
        'action_str': ACTION_TYPES.get(action, f"Unknown({action})"),
        'sync_type': sync_type,
        'sync_type_str': sync_type_str,
        'error_code': error_code,
        'error_str': error_str,
        'has_errors': error_code != 0,
        'is_sync': action == 3
    }

class SamplerateError(Exception):
    pass

class ChannelError(Exception):
    pass

class Decoder(srd.Decoder):
    api_version = 3
    id = 'madelink'
    name = 'Madelink'
    longname = 'Madelink Custom UART Protocol'
    desc = 'Custom UART protocol with 3-byte header + COBS encoded payloads.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = []
    tags = ['Embedded/industrial']
    channels = (
        {'id': 'mdl_channel', 'type': 209, 'name': 'Channel', 'desc': 'madelink channel', 'idn':'dec_0uart_chan_mdl_channel'},
    )
    options = (
        {'id': 'baudrate', 'desc': 'Baud rate', 'default': 115200, 'idn':'dec_0uart_opt_baudrate'},
        {'id': 'sub_decoder', 'desc': 'Property decoder', 'default': 'default',
            'values': tuple(get_available_sub_decoders()), 'idn':'dec_madelink_opt_sub_decoder'},
    )
    annotations = (
        ('108', 'header', 'Header with parsed fields'),
        ('7', 'payload', 'COBS payload'),
        ('6', 'decoded', 'Decoded payload'),
        ('0', 'error', 'Protocol error'),
        ('1', 'delimiter', 'COBS delimiter'),
        ('1000', 'warnings', 'Warnings'),
        ('200', 'completion', 'Frame completion'),
        ('300', 'sync_header', 'SYNC header (1 byte)'),
    )
    annotation_rows = (
        ('protocol', 'Madelink', (0, 1, 2, 3, 4, 6, 7)),
        ('warnings', 'Warnings', (5,)),
    )
    idle_state = 'WAIT FOR START BIT'

    def putx(self, data):
        s, halfbit = self.startsample, self.bit_width / 2.0
        # Always use frame-based display (anno_startstop = 'no')
        self.put(self.frame_start, self.samplenum + ceil(halfbit * (1+1.0)), self.out_ann, data)  # Use 1.0 stop bit default

    def putg(self, data):
        s, halfbit = self.samplenum, self.bit_width / 2.0
        self.put(s - floor(halfbit), s + ceil(halfbit), self.out_ann, data)

    def putgse(self, ss, es, data):
        self.put(ss, es, self.out_ann, data)

    def __init__(self):
        self.reset()
        self.sub_decoder = None
        self.property_ids = {}

    def reset(self):
        self.samplerate = None
        self.samplenum = 0
        self.frame_start = -1
        self.frame_valid = None
        self.startbit = -1
        self.cur_data_bit = 0
        self.datavalue = 0
        self.paritybit = -1
        self.stopbit1 = -1
        self.startsample = -1
        self.state = 'WAIT FOR START BIT'
        self.databits = []
        
        # Madelink protocol state
        self.protocol_state = 'WAIT_HEADER'
        self.header_bytes = []
        self.payload_buffer = []
        self.byte_start_sample = -1
        self.header_start_sample = -1
        self.payload_start_sample = -1
        self.header_info = None
        self.expected_payloads = 0
        self.received_payloads = 0

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.bw = (8 + 7) // 8  # Use 8 data bits as default
        
        # Load the selected sub-decoder
        decoder_name = self.options.get('sub_decoder', 'default')
        self.sub_decoder = load_sub_decoder(decoder_name)
        self.property_ids = getattr(self.sub_decoder, 'PROPERTY_IDS', {})
    
    def reset_protocol_state(self):
        """Reset only Madelink protocol state, not UART state."""
        self.protocol_state = 'WAIT_HEADER'
        self.header_bytes = []
        self.payload_buffer = []
        self.header_info = None
        self.expected_payloads = 0
        self.received_payloads = 0
        self.header_start_sample = -1
        self.payload_start_sample = -1
    


    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value
            # The width of one UART bit in number of samples.
            self.bit_width = float(self.samplerate) / float(self.options['baudrate'])

    def get_sample_point(self, bitnum):
        # Determine absolute sample number of a bit slot's sample point.
        # bitpos is the samplenumber which is in the middle of the
        # specified UART bit (0 = start bit, 1..x = data, x+1 = parity bit
        # (if used) or the first stop bit, and so on).
        # The samples within bit are 0, 1, ..., (bit_width - 1), therefore
        # index of the middle sample within bit window is (bit_width - 1) / 2.
        bitpos = self.frame_start + (self.bit_width - 1) / 2.0
        bitpos += bitnum * self.bit_width
        return bitpos

    def wait_for_start_bit(self, signal):
        # Save the sample number where the start bit begins.
        self.frame_start = self.samplenum
        self.frame_valid = True

        self.state = 'GET START BIT'

    def get_start_bit(self, signal):
        self.startbit = signal

        # The startbit must be 0. If not, ignore this byte and wait
        # for the next start bit (assuming this one was spurious).
        if self.startbit != 0:
            # Ignore this byte - don't show frame error annotation
            self.frame_valid = False
            es = self.samplenum + ceil(self.bit_width / 2.0)
            self.state = 'WAIT FOR START BIT'
            return

        self.cur_data_bit = 0
        self.datavalue = 0
        self.startsample = -1

        # Start bit display is disabled by default

        self.state = 'GET DATA BITS'

    def get_data_bits(self, signal):
        # Save the sample number of the middle of the first data bit.
        if self.startsample == -1:
            self.startsample = self.samplenum

        # Store individual data bits and their start/end samplenumbers.
        s, halfbit = self.samplenum, int(self.bit_width / 2)
        self.databits.append([signal, s - halfbit, s + halfbit])

        # Return here, unless we already received all data bits.
        self.cur_data_bit += 1
        if self.cur_data_bit < 8:  # Use 8 data bits as default
            return

        # Convert accumulated data bits to a data value.
        bits = [b[0] for b in self.databits]
        # Use lsb-first as default (no bit reversal needed)
        self.datavalue = bitpack(bits)
        
        # Store the start sample of this byte for annotation
        if self.byte_start_sample == -1:
            self.byte_start_sample = self.databits[0][1]  # Start of first data bit
        
        # Raw byte display is disabled by default
        
        # Don't process the byte for Madelink protocol yet - wait until frame is validated
       
        self.databits = []

        # Advance to reception of STOP bits (parity disabled by default)
        self.state = 'GET STOP BITS'
  
    def get_parity_bit(self, signal):
        self.paritybit = signal

        if parity_ok('none', self.paritybit,
                     self.datavalue, 8):  # Use none parity and 8 data bits as default
            self.putg([2, ['Parity bit', 'Parity', 'P']])
        else:
            # Parity error - ignore this byte (don't show error annotation)
            self.frame_valid = False

        self.state = 'GET STOP BITS'

    # TODO: Currently only supports 1 stop bit.
    def get_stop_bits(self, signal):
        self.stopbit1 = signal

        # Stop bits must be 1. If not, ignore this byte.
        if self.stopbit1 != 1:
            # Ignore this byte - don't show frame error annotation
            self.frame_valid = False

        # Stop bit display is disabled by default

        # Pass the complete UART frame to upper layers.
        es = self.samplenum + ceil(self.bit_width / 2.0)

        # Only process the byte for Madelink protocol if frame is valid
        if self.frame_valid:
            self.process_madelink_payload_byte(self.datavalue)

        # Reset byte start sample tracking
        self.byte_start_sample = -1
        self.state = 'WAIT FOR START BIT'
    
    def process_madelink_payload_byte(self, byte_value):
        """Process a received byte according to Madelink protocol."""
        byte_end_sample = self.samplenum + ceil(self.bit_width / 2.0)
        
        if self.protocol_state == 'WAIT_HEADER':
            # Check if this is a SYNC header (1 byte) by examining action bits
            action = (byte_value >> 6) & 0x03
            
            if action == 3:  # SYNC action
                # This is a 1-byte SYNC header
                self.header_info = parse_sync_header(byte_value)
                
                if self.header_info and self.header_info['is_sync']:
                    # Show SYNC header information with different color
                    error_indicator = "!" if self.header_info['has_errors'] else "OK"
                    detail_str = (f"[SYNC] [{self.header_info['sync_type_str']}] "
                                f"[{self.header_info['error_str']}] [{error_indicator}]")
                    
                    self.putgse(self.byte_start_sample, byte_end_sample,
                               [7, [f"SYNC Header: {byte_value:02X} - {detail_str}", 
                                    f"SYNC: {detail_str}", 
                                    f"{byte_value:02X}"]])
                    
                    # SYNC headers don't have payloads, wait for next header
                    self.protocol_state = 'WAIT_HEADER'
                    self.header_bytes = []
                    self.expected_payloads = 0
                    self.received_payloads = 0
                else:
                    # SYNC header parsing failed
                    self.putgse(self.byte_start_sample, byte_end_sample,
                               [3, ['SYNC Parse Error', 'SYNC Err', 'Err']])
            else:
                # This is a regular 3-byte header
                if len(self.header_bytes) == 0:
                    # First header byte - store the start sample
                    self.header_start_sample = self.byte_start_sample
                
                self.header_bytes.append(byte_value)
                self.putgse(self.byte_start_sample, byte_end_sample, 
                           [0, ['H%d: 0x%02X' % (len(self.header_bytes), byte_value), 
                                'H%d: %02X' % (len(self.header_bytes), byte_value)]])
                
                if len(self.header_bytes) == 3:
                    # Header complete, parse it
                    self.header_info = parse_madelink_header(self.header_bytes, self.property_ids)
                    
                    if self.header_info:
                        # Show detailed header information
                        header_str = ' '.join(['%02X' % b for b in self.header_bytes])
                        parity_status = "OK" if self.header_info['parity_valid'] else "ERR"
                        
                        # Detailed annotation with all fields
                        detail_str = (f"[{self.header_info['action_str']}] "
                                    f"[{self.header_info['property_str']}] "
                                    f"[cnt:{self.header_info['node_cnt']}] "
                                    f"[{self.header_info['staging_error_str']}] "
                                    f"[Par:{parity_status}]")

                        self.putgse(self.header_start_sample, byte_end_sample,
                                   [0, [f"Header: {header_str} - {detail_str}", 
                                        f"HDR: {detail_str}", 
                                        f"{header_str}"]])
                        
                        # Set expected payload count
                        if self.header_info['action'] == 2:  # BROADCAST action
                            # BROADCAST always expects exactly 1 payload regardless of node_cnt
                            self.expected_payloads = 1
                        else:
                            # Other actions use node_cnt for payload count
                            self.expected_payloads = self.header_info['node_cnt']
                        self.received_payloads = 0
                        
                        if self.expected_payloads > 0:
                            self.protocol_state = 'COLLECT_PAYLOAD'
                            self.payload_buffer = []
                            self.payload_start_sample = -1
                        else:
                            # No payloads expected, wait for next header
                            self.protocol_state = 'WAIT_HEADER'
                            self.header_bytes = []
                    else:
                        # Header parsing failed
                        self.putgse(self.header_start_sample, byte_end_sample,
                                   [3, ['Header Parse Error', 'HDR Err', 'Err']])
                        self.protocol_state = 'WAIT_HEADER'
                        self.header_bytes = []
                
        elif self.protocol_state == 'COLLECT_PAYLOAD':
            # Track payload start
            if len(self.payload_buffer) == 0:
                self.payload_start_sample = self.byte_start_sample
            
            # Collecting COBS encoded payload
            self.payload_buffer.append(byte_value)
            
            if byte_value == 0:
                # COBS delimiter found, decode the payload
                self.putgse(self.byte_start_sample, byte_end_sample,
                           [4, ['COBS End', 'End', '0']])
                
                if len(self.payload_buffer) > 1:  # More than just the delimiter
                    payload_to_decode = self.payload_buffer[:-1]  # Remove the delimiter
                    decoded, success = cobs_decode(payload_to_decode)
                    
                    if success and decoded:
                        # Validate checksum - last byte should make sum equal 0
                        checksum_valid = False
                        if len(decoded) > 0:
                            # Calculate sum of all bytes including checksum
                            total_sum = sum(decoded) & 0xFF  # Keep as 8-bit
                            checksum_valid = (total_sum == 0)
                        
                        # Show raw COBS data
                        cobs_str = ' '.join(['%02X' % b for b in payload_to_decode])
                        self.putgse(self.payload_start_sample, self.byte_start_sample,
                                   [1, ['COBS: %s' % cobs_str, 'COBS']])
                        
                        # Show decoded data with checksum status
                        if len(decoded) > 1:
                            # Split data and checksum for display
                            data_bytes = decoded[:-1]
                            checksum_byte = decoded[-1]
                            checksum_status = "OK" if checksum_valid else "ERR"
                            
                            # Use sub-decoder for payload interpretation
                            if self.sub_decoder and hasattr(self.sub_decoder, 'decode_payload') and self.header_info:
                                try:
                                    decoded_payload = self.sub_decoder.decode_payload(
                                        self.header_info['property_id'], 
                                        data_bytes, 
                                        checksum_byte
                                    )
                                    payload_display = decoded_payload.get('decoded', '')
                                    payload_info = decoded_payload.get('info', '')
                                                                        
                                    self.putgse(self.payload_start_sample, self.byte_start_sample,
                                               [2, [f'Data: {payload_display} CS:{checksum_byte:02X} {checksum_status}', 
                                                    f'Dec: {payload_display} CS:{checksum_status}',
                                                    f'{payload_info}']])
                                except Exception as e:
                                    # Fallback to basic display on decoder error
                                    data_str = ' '.join(['%02X' % b for b in data_bytes])
                                    data_ascii = ''.join([chr(b) if 32 <= b <= 126 else '.' for b in data_bytes])
                                    self.putgse(self.payload_start_sample, self.byte_start_sample,
                                               [2, ['Data: %s (%s) CS:%02X%s' % (data_str, data_ascii, checksum_byte, checksum_status), 
                                                    'Dec: %s CS:%s' % (data_str, checksum_status)]])
                            else:
                                # Fallback to basic display
                                data_str = ' '.join(['%02X' % b for b in data_bytes])
                                data_ascii = ''.join([chr(b) if 32 <= b <= 126 else '.' for b in data_bytes])
                                self.putgse(self.payload_start_sample, self.byte_start_sample,
                                           [2, ['Data: %s (%s) CS:%02X%s' % (data_str, data_ascii, checksum_byte, checksum_status), 
                                                'Dec: %s CS:%s' % (data_str, checksum_status)]])
                        else:
                            # Single byte (just checksum)
                            checksum_status = "OK" if checksum_valid else "ERR"
                            self.putgse(self.payload_start_sample, self.byte_start_sample,
                                       [2, ['Checksum: %02X%s' % (decoded[0], checksum_status), 
                                            'CS:%s' % checksum_status]])
                        
                        # Show error if checksum is invalid
                        if not checksum_valid:
                            self.putgse(self.payload_start_sample, self.byte_start_sample,
                                       [3, ['Checksum Error (Sum: 0x%02X)' % ((sum(decoded) & 0xFF)), 
                                            'CS Err', 'CS ERR']])
                    else:
                        # COBS decode error
                        self.putgse(self.payload_start_sample, byte_end_sample,
                                   [3, ['COBS Decode Error', 'COBS Err', 'Err']])
                else:
                    # Empty payload (just the delimiter)
                    self.putgse(self.payload_start_sample, self.byte_start_sample,
                               [3, ['Empty Payload', 'Empty', 'E']])
                
                # Reset for next payload
                self.payload_buffer = []
                self.received_payloads += 1
                
                # Check if we've received all expected payloads
                if self.received_payloads >= self.expected_payloads:
                    # All payloads received, wait for next header
                    self.protocol_state = 'WAIT_HEADER'
                    self.header_bytes = []
                    
                    # Show completion status
                    completion_str = f"Frame Complete: {self.received_payloads}/{self.expected_payloads} payloads"
                    self.putgse(byte_end_sample - ceil(self.bit_width), byte_end_sample,
                               [2, [completion_str, f"Complete {self.received_payloads}/{self.expected_payloads}", "Done"]])
            else:
                # Regular payload byte - don't show individual bytes, wait for complete payload
                pass

    def get_wait_cond(self, inv):
        # Return condititions that are suitable for Decoder.wait(). Those
        # conditions either match the falling edge of the START bit, or
        # the sample point of the next bit time.
        state = self.state
        if state == 'WAIT FOR START BIT':
            return {0: 'r' if inv else 'f'}
        if state == 'GET START BIT':
            bitnum = 0
        elif state == 'GET DATA BITS':
            bitnum = 1 + self.cur_data_bit
        elif state == 'GET PARITY BIT':
            bitnum = 1 + 8  # Use 8 data bits as default
        elif state == 'GET STOP BITS':
            bitnum = 1 + 8  # Use 8 data bits as default
            bitnum += 0  # Use 'none' parity as default
        want_num = ceil(self.get_sample_point(bitnum))
        return {'skip': want_num - self.samplenum}

    def inspect_sample(self, signal, inv):
        # Inspect a sample returned by .wait() for the specified UART line.
        if inv:
            signal = not signal

        state = self.state
        if state == 'WAIT FOR START BIT':
            self.wait_for_start_bit(signal)
        elif state == 'GET START BIT':
            self.get_start_bit(signal)
        elif state == 'GET DATA BITS':
            self.get_data_bits(signal)
        elif state == 'GET PARITY BIT':
            self.get_parity_bit(signal)
        elif state == 'GET STOP BITS':
            self.get_stop_bits(signal)

    def decode(self):
        if not self.samplerate:
            raise SamplerateError('Cannot decode without samplerate.')

        inv = False  # Signal inversion disabled by default

        while True:
            conds = self.get_wait_cond(inv)
            (mdl_channel, ) = self.wait(conds)
            if (self.matched & (0b1 << 0)):
                self.inspect_sample(mdl_channel, inv)
