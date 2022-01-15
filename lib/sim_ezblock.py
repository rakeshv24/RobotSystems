class I2C(object):
    MASTER = 0
    SLAVE  = 1
    RETRY = 5

    def __init__(self, *args, **kargs):
        super().__init__()
        
    def _i2c_write_byte(self, addr, data):
        return 1
    
    def _i2c_write_byte_data(self, addr, reg, data):
        return 1
    
    def _i2c_write_word_data(self, addr, reg, data):
        return 1

    def _i2c_write_i2c_block_data(self, addr, reg, data):
        return 1
    
    def _i2c_read_byte(self, addr):
        return 1
    
    def _i2c_read_i2c_block_data(self, addr, reg, num):
        return 1
    
    def is_ready(self, addr):
        return 1

    def scan(self):
        return 1

    def send(self, send, addr, timeout=0):
        pass

    def recv(self, recv, addr=0x00, timeout=0):
        return 1

    def mem_write(self, data, addr, memaddr, timeout=5000, addr_size=8):
        pass
    
    def mem_read(self, data, addr, memaddr, timeout=5000, addr_size=8):
        return 1
    
    def readfrom_mem_into(self, addr, memaddr, buf):
        return 1
    
    def writeto_mem(self, addr, memaddr, data):
        pass
    
    
timer = [
    {
        "arr": 0
    }
] * 4


class PWM(I2C):
    REG_CHN = 0x20
    REG_FRE = 0x30
    REG_PSC = 0x40
    REG_ARR = 0x44

    ADDR = 0x14

    CLOCK = 72000000

    def __init__(self, channel, debug="critical"):
        super().__init__()
    
    def i2c_write(self, reg, value):
        pass

    def freq(self, *freq):
        return 1
        
    def prescaler(self, *prescaler):
        return 1

    def period(self, *arr):
        return 0
        
    def pulse_width(self, *pulse_width):
        return 1
    
    def pulse_width_percent(self, *pulse_width_percent):
        return 1

class Servo(object):
    MAX_PW = 2500
    MIN_PW = 500
    _freq = 50
    def __init__(self, pwm):
        super().__init__()
        
    def map(self, x, in_min, in_max, out_min, out_max):
        return 1
        
    # angle ranges -90 to 90 degrees
    def angle(self, angle):
        pass


class Pin(object):
    # OUT = GPIO.OUT
    # IN = GPIO.IN
    # IRQ_FALLING = GPIO.FALLING
    # IRQ_RISING = GPIO.RISING
    # IRQ_RISING_FALLING = GPIO.BOTH
    # PULL_UP = GPIO.PUD_UP
    # PULL_DOWN = GPIO.PUD_DOWN
    PULL_NONE = None

    _dict = {
        "BOARD_TYPE": 12,
    }

    _dict_1 = {
        "D0":  17,
        "D1":  18,
        "D2":  27,
        "D3":  22,
        "D4":  23,
        "D5":  24,
        "D6":  25,
        "D7":  4,
        "D8":  5,
        "D9":  6,
        "D10": 12,
        "D11": 13,
        "D12": 19,
        "D13": 16,
        "D14": 26,
        "D15": 20,
        "D16": 21,
        "SW":  19,
        "LED": 26,
        "BOARD_TYPE": 12,
        "RST": 16,
        "BLEINT": 13,
        "BLERST": 20,
        "MCURST": 21,
    }

    _dict_2 = {
        "D0":  17,
        "D1":   4, # Changed
        "D2":  27,
        "D3":  22,
        "D4":  23,
        "D5":  24,
        "D6":  25, # Removed
        "D7":   4, # Removed
        "D8":   5, # Removed
        "D9":   6,
        "D10": 12,
        "D11": 13,
        "D12": 19,
        "D13": 16,
        "D14": 26,
        "D15": 20,
        "D16": 21,
        "SW":  25, # Changed
        "LED": 26,
        "BOARD_TYPE": 12,
        "RST": 16,
        "BLEINT": 13,
        "BLERST": 20,
        "MCURST":  5, # Changed
    }

    def __init__(self, *value):
        super().__init__()
        
    def check_board_type(self):
        pass
        
    def init(self, mode, pull=PULL_NONE):
        pass

    def dict(self, *_dict):
        pass

    def __call__(self, value):
        return self.value(value)

    def value(self, *value):
        pass
        
    def on(self):
        return self.value(1)

    def off(self):
        return self.value(0)

    def high(self):
        return self.on()

    def low(self):
        return self.off()

    def mode(self, *value):
        return 0
    
    def pull(self, *value):
        return self._pull

    def irq(self, handler=None, trigger=None, bouncetime=200):
        pass

    def name(self):
        return "name"

    def names(self):
        return [self.name, self._board_name]


class ADC(I2C):
    ADDR=0x14                   

    def __init__(self, chn):    
        super().__init__()
        
    def read(self):             
        return 1

    def read_voltage(self):
        return 1