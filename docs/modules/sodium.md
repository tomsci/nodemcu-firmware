# Sodium module
| Since  | Origin / Contributor  | Maintainer  | Source  |
| :----- | :-------------------- | :---------- | :------ |
| 2018-10-27 | [Tom Sutcliffe](https://github.com/tomsci) | [Tom Sutcliffe](https://github.com/tomsci) | [sodium.c](../../components/modules/sodium.c)|

This module wraps the [LibSodium](https://libsodium.org/) C library. LibSodium is a library for performing Elliptic Curve Cryptography.

In addition to the flag for enabling this module during ROM build, `Component config -> NodeMCU Modules -> Sodium module`, there are additional settings for libsodium under `Component config -> libsodium`.

!!! note

    Almost all functions in this module require a working random number generator. On the ESP32 this normally means that *WiFi must be started* otherwise ALL OF THE CRYPTOGRAPHY WILL SILENTLY BE COMPROMISED. Make sure to call `wifi.start()` before any of the functions in this module. See the [Espressif documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/random.html) for more information. The only exception is `sodium.crypto_box.seal_open()` which does not require a random number source to operate. The non-crypto functions in `sodium.random` can be used with WiFi off, providing that pseudo-randomness is acceptable for the purposes they are being used for.

# Random number generation
See also [https://download.libsodium.org/doc/generating_random_data](https://download.libsodium.org/doc/generating_random_data). On the esp32, a custom implementation `esp_random()` is used which (usually) requires WiFi being enabled to be truly random.

## sodium.random.random()
Returns a random signed 32-bit integer between `INT32_MIN` and `INT32_MAX`. On builds with 64-bit integer support (`5.3-int64-xxx`), you can convert this to an unsigned 32-bit integer with `result % (1 << 32)`, or on builds with only 64-bit double support (`5.1-doublefp`, `5.3-int32-doublefp`), with `result % (2 ^ 32)`.

If WiFi is not started by calling `wifi.start()` before calling this function, then the result will only be pseudo-random, not truly random.

#### Syntax
`sodium.random.random()`

#### Parameters
None

#### Returns
A uniformly-distributed random 32-bit integer.

## sodium.random.uniform()
Returns a random integer `0 <= result < upper_bound`. Unlike `sodium.random.random() % upper_bound`, it guarantees a uniform distribution of the possible output values even when `upper_bound` is not a power of 2. Specifying an `upper_bound > 0x7FFFFFFF` is not recommended because the behavior will vary depending on the Lua build configuration.

If WiFi is not started by calling `wifi.start()` before calling this function, then the result will only be pseudo-random, not truly random.

#### Syntax
`sodium.random.uniform(upper_bound)`

#### Parameters
- `upper_bound` an integer.

#### Returns
An integer `>= 0` and `< upper_bound`

## sodium.random.buf()
Generates `n` bytes of random data. If WiFi is not started by calling `wifi.start()` before calling this function, then the result will only be pseudo-random, not truly random.

#### Syntax
`sodium.random.buf(n)`

#### Parameters
- `n` number of bytes to return.

#### Returns
A string of `n` random bytes.

# Generating public and secret keys
The keys created by `crypto_box.keypair()` can be used the `crypto_box.seal*()` functions.

## sodium.crypto_box.keypair()
Generates a new keypair. Wifi must be started, by calling `wifi.start()`, before calling this function.

#### Syntax
`sodium.crypto_box.keypair()`

#### Parameters
None

#### Returns
Two values, `public_key, secret_key`. Both are strings (although containing non-printable characters).

#### Example
```lua
public_key, secret_key = sodium.crypto_box.keypair()
```

# Sealed box public key cryptography
See also [https://download.libsodium.org/doc/public-key_cryptography/sealed_boxes](https://download.libsodium.org/doc/public-key_cryptography/sealed_boxes).

## sodium.crypto_box.seal()
Encrypts a message using a public key, such that only someone knowing the corresponding secret key can decrypt it using [`sodium.crypto_box.seal_open()`](#sodiumcryptoboxsealopen). This API does not store any information about who encrypted the message, therefore at the point of decryption there is is no proof the message hasn't been tampered with or sent by somone else. Wifi must be started, by calling `wifi.start()`, before calling this function.

#### Syntax
`sodium.crypto_box.seal(message, public_key)`

#### Parameters
- `message` - the string to encrypt.
- `public_key` - the public key to encrypt with.

#### Returns
The encrypted message, as a string. Errors if `public_key` is not a valid public key as returned by `sodium.crypto_box.keypair()` or if the message could not be encrypted.

#### Example
```lua
ciphertext = sodium.crypto_box.seal(message, public_key)
```

## sodium.crypto_box.seal_open()
Decrypts a message encrypted with [`crypto_box.seal()`](#sodiumcryptoboxseal).

#### Syntax
`sodium.crypto_box.seal_open(ciphertext, public_key, secret_key)`

#### Parameters
- `ciphertext` - the encrypted message.
- `public_key` - the public key the message was encrypted with.
- `secret_key` - the secret key corresponding to the specified public key.

#### Returns
The decrypted plain text of the message. Returns `nil` if the `ciphertext` could not be decrypted.

#### Example
```lua
message = sodium.crypto_box.seal_open(ciphertext, public_key, secret_key)
```

# Hashing
See also [https://download.libsodium.org/doc/hashing/generic_hashing](https://download.libsodium.org/doc/hashing/generic_hashing).

The hash functions are implemented using BLAKE2b, a simple, standardized ([RFC 7693](https://www.rfc-editor.org/rfc/rfc7693.txt)) secure hash function that is as strong as SHA-3 but faster than SHA-1 and MD5.

## sodium.generichash()
Computes a hash of the given data. The same data will always produce the same hash (providing `key` and `out_len` parameters are the same).

#### Syntax
`sodium.generichash(data[, key[, out_len]])`

#### Parameters
- `data` - the data to hash
- `key` - Optional. A string to use as the key. A key parameter can be used for example to make sure that different applications generate different hashes even if they process the same data. If specified, must be between 16 and 64 bytes.
- `out_len` - Optional. The length of hash to output, which must be between 16 and 64. If not specified, defaults to 32 bytes (`crypto_generichash_BYTES`).

#### Returns
The hash of the data, as an unencoded string of `out_len` bytes. Use something like `encoder.toHex(result)` if you need it as ASCII text.

#### Example
```lua
hash = sodium.generichash("Some data")
print("Result:", encoder.toHex(hash))
```

## sodium.generichash_init()
Provides a way to calculate a hash in chunks, so that not all the data needs to be in memory at once.

#### Syntax
`sodium.generichash_init([key[, out_len]])`

#### Parameters
- `key` - Optional. As per sodium.generichash().
- `out_len` - Optional. As per sodium.generichash().

#### Returns
An object with 2 methods, `update(data)` and `final()`. Call `update()` with each data chunk in turn, then call `final()` to fetch the resulting hash. Do not call `update()` again after calling `final()`: to calculate another hash, call `generichash_init()` again to get a new object.

#### Example
```lua
hasher = sodium.generichash_init()
hasher:update(data_chunk_1)
hasher:update(data_chunk_2)
hash = hasher:final()
print("Result:", encoder.toHex(hash))
```
