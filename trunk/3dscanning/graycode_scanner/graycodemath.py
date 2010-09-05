def get_bit(value, bit_num):
    return value & (1 << bit_num)

def decode_gray_code(gray_code):
    for i in range(4, -1, -1):
        gray_code = gray_code ^ (gray_code//(2 ** (2 ** i)))
    return gray_code    

def generate_gray_code(binary):
    return binary ^ binary >> 1
