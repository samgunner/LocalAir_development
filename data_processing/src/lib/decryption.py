import json

from speck import SpeckCipher


class SpeckDecryptor:
    def __init__(self, key, block_size, key_size=None):
        # key can either be an integer, bytes or iterable of ints
        match key:
            case int(key):
                if key_size is None:
                    raise TypeError('When key is an int, key_size is required')
            case bytes(key) | [*key]:
                if key_size is None:
                    key_size = len(key) * 8
                key = int.from_bytes(bytes(key), 'big')
            case _:
                raise ValueError('key must be an int, bytes, or iterable of ints')
        
        self.cipher = SpeckCipher(key, key_size=key_size, block_size=block_size)
        self.block_size_bytes = block_size // 8

    def decrypt_data(self, data):
        message = b''.join(self._decrypt_chunk(chunk) for chunk in self._split_into_chunks(data))
        return message.split(b'\00')[0]

    def _decrypt_chunk(self, chunk):
        encrypted_int = int.from_bytes(chunk, byteorder='big')
        decrypted_int = self.cipher.decrypt(encrypted_int)
        return decrypted_int.to_bytes(length=self.block_size_bytes, byteorder='big')

    def _split_into_chunks(self, data):
        for i in range(0, len(data), self.block_size_bytes):
            yield data[i:i + self.block_size_bytes]


class LocalAirDecryptor(SpeckDecryptor):
    def decrypt_data(self, ascii_data):
        data = bytes.fromhex(ascii_data)
        message = super().decrypt_data(data)
        
        if not message or message[:1] != b'{':
            raise ValueError('Failed to decrypt message, corrupted data or invalid key?')

        try:
            message = message.decode('ascii')
        except UnicodeDecodeError:
            raise ValueError('Failed to decode message, corrupted data?')

        try:
            return json.loads(message)
        except json.JSONDecodeError:
            raise ValueError('Failed to parse JSON, corrupted data?')
