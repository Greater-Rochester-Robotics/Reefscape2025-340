/**
 * MessagePack serializer and deserializer.
 * Adapted from https://github.com/gerth2/NetworkTablesClients/blob/main/nt4/js/src/msgpack/msgpack.js
 */
export class MessagePack {
    private constructor() {}

    /**
     * Serialize data into a MessagePack byte array.
     * @param data The data to serialize.
     * @param options Serialization options.
     * @returns The serialized byte array.
     */
    public static serialize(
        data: any[],
        options?: { multiple?: boolean; typeHint?: string; invalidTypeReplacement?: (data: any) => any },
    ): Uint8Array {
        if (options && options.multiple && !Array.isArray(data)) {
            throw new TypeError(`Invalid argument type: Expected an Array to serialize multiple values.`);
        }

        let array = new Uint8Array(128);

        const th = options?.typeHint ? options.typeHint : ``;
        let floatBuffer: ArrayBuffer, floatView: DataView;
        let length = 0;
        const pow32 = 0x100000000;

        if (options?.multiple) {
            for (let i = 0; i < data.length; i++) append(data[i], false, th);
        } else {
            append(data, false, th);
        }

        return array.subarray(0, length);

        /**
         * Append data to the byte array.
         * @param data The data to append.
         * @param isReplacement If an invalid type should fall through the invalidTypeReplacement callback specified in options.
         * @param th Type hint for numbers.
         */
        function append(data: any, isReplacement?: boolean, th?: string): void {
            switch (typeof data) {
                case `undefined`:
                    appendNull();
                    break;
                case `boolean`:
                    appendBoolean(data);
                    break;
                case `number`:
                    appendNumber(data, th);
                    break;
                case `string`:
                    appendString(data);
                    break;
                case `object`:
                    if (data === null) appendNull();
                    else if (data instanceof Date) appendDate(data);
                    else if (Array.isArray(data)) appendArray(data);
                    else if (data instanceof Uint8Array || data instanceof Uint8ClampedArray) appendBinaryArray(data);
                    else if (
                        data instanceof Int8Array ||
                        data instanceof Int16Array ||
                        data instanceof Uint16Array ||
                        data instanceof Int32Array ||
                        data instanceof Uint32Array ||
                        data instanceof Float32Array ||
                        data instanceof Float64Array
                    )
                        appendArray(data);
                    else appendObject(data);
                    break;
                default:
                    if (!isReplacement && options && options.invalidTypeReplacement) {
                        if (typeof options.invalidTypeReplacement === `function`) append(options.invalidTypeReplacement(data), true, th);
                        else append(options.invalidTypeReplacement, true, th);
                    } else {
                        throw new Error(`Invalid argument type: The type '${typeof data}' cannot be serialized.`);
                    }
            }
        }

        /**
         * Append a null byte to the array.
         */
        function appendNull(): void {
            appendByte(0xc0);
        }

        /**
         * Append a boolean to the array.
         * @param data The boolean to append.
         */
        function appendBoolean(data: boolean): void {
            appendByte(data ? 0xc3 : 0xc2);
        }

        /**
         * Append a number to the array.
         * @param data The number to append.
         * @param th Type hint. Should be int, double, or float.
         */
        function appendNumber(data: number, th?: string): void {
            const isInteger = th === `int` || (isFinite(data) && Math.floor(data) === data && th !== `double` && th !== `float`);
            if (isInteger) {
                if (data >= 0 && data <= 0x7f) {
                    appendByte(data);
                } else if (data < 0 && data >= -0x20) {
                    appendByte(data);
                } else if (data > 0 && data <= 0xff) {
                    // uint8
                    appendBytes([0xcc, data]);
                } else if (data >= -0x80 && data <= 0x7f) {
                    // int8
                    appendBytes([0xd0, data]);
                } else if (data > 0 && data <= 0xffff) {
                    // uint16
                    appendBytes([0xcd, data >>> 8, data]);
                } else if (data >= -0x8000 && data <= 0x7fff) {
                    // int16
                    appendBytes([0xd1, data >>> 8, data]);
                } else if (data > 0 && data <= 0xffffffff) {
                    // uint32
                    appendBytes([0xce, data >>> 24, data >>> 16, data >>> 8, data]);
                } else if (data >= -0x80000000 && data <= 0x7fffffff) {
                    // int32
                    appendBytes([0xd2, data >>> 24, data >>> 16, data >>> 8, data]);
                } else if (data > 0 && data <= 0xffffffffffffffff) {
                    // uint64
                    const hi = data / pow32;
                    const lo = data % pow32;
                    appendBytes([0xd3, hi >>> 24, hi >>> 16, hi >>> 8, hi, lo >>> 24, lo >>> 16, lo >>> 8, lo]);
                } else if (data >= -0x8000000000000000 && data <= 0x7fffffffffffffff) {
                    // int64
                    appendByte(0xd3);
                    appendInt64(data);
                } else if (data < 0) {
                    // below int64
                    appendBytes([0xd3, 0x80, 0, 0, 0, 0, 0, 0, 0]);
                } else {
                    // above uint64
                    appendBytes([0xcf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]);
                }
            } else {
                if (!floatView) {
                    floatBuffer = new ArrayBuffer(8);
                    floatView = new DataView(floatBuffer);
                }
                floatView.setFloat64(0, data);
                appendByte(0xcb);
                appendBytes(new Uint8Array(floatBuffer));
            }
        }

        /**
         * Append a string to the array.
         * @param data The string to append.
         */
        function appendString(data: string): void {
            const bytes = encodeUtf8(data);
            const length = bytes.length;

            if (length <= 0x1f) appendByte(0xa0 + length);
            else if (length <= 0xff) appendBytes([0xd9, length]);
            else if (length <= 0xffff) appendBytes([0xda, length >>> 8, length]);
            else appendBytes([0xdb, length >>> 24, length >>> 16, length >>> 8, length]);

            appendBytes(bytes);
        }

        /**
         * Append an array to the array.
         * @param data The array to append.
         */
        function appendArray(
            data: any[] | Int8Array | Int16Array | Uint16Array | Int32Array | Uint32Array | Float32Array | Float64Array,
        ): void {
            const length = data.length;

            if (length <= 0xf) appendByte(0x90 + length);
            else if (length <= 0xffff) appendBytes([0xdc, length >>> 8, length]);
            else appendBytes([0xdd, length >>> 24, length >>> 16, length >>> 8, length]);

            for (let index = 0; index < length; index++) {
                append(data[index]);
            }
        }

        /**
         * Append a binary array to the array.
         * @param data The binary array to append.
         */
        function appendBinaryArray(data: Uint8Array | Uint8ClampedArray): void {
            const length = data.length;

            if (length <= 0xf) appendBytes([0xc4, length]);
            else if (length <= 0xffff) appendBytes([0xc5, length >>> 8, length]);
            else appendBytes([0xc6, length >>> 24, length >>> 16, length >>> 8, length]);

            appendBytes(data);
        }

        /**
         * Append an object to the array.
         * @param data The object to append.
         */
        function appendObject(data: Record<string, any>): void {
            let length = 0;
            for (const key in data) {
                if (data[key] !== undefined) {
                    length++;
                }
            }

            if (length <= 0xf) appendByte(0x80 + length);
            else if (length <= 0xffff) appendBytes([0xde, length >>> 8, length]);
            else appendBytes([0xdf, length >>> 24, length >>> 16, length >>> 8, length]);

            for (const key in data) {
                const value = data[key];
                if (value !== undefined) {
                    append(key);
                    append(value);
                }
            }
        }

        /**
         * Append a date to the array.
         * @param data The date to append.
         */
        function appendDate(data: Date): void {
            const sec = data.getTime() / 1000;
            if (data.getMilliseconds() === 0 && sec >= 0 && sec < 0x100000000) {
                // 32 bit seconds.
                appendBytes([0xd6, 0xff, sec >>> 24, sec >>> 16, sec >>> 8, sec]);
            } else if (sec >= 0 && sec < 0x400000000) {
                // 30 bit nanoseconds, 34 bit seconds.
                const ns = data.getMilliseconds() * 1000000;
                appendBytes([
                    0xd7,
                    0xff,
                    ns >>> 22,
                    ns >>> 14,
                    ns >>> 6,
                    ((ns << 2) >>> 0) | (sec / pow32),
                    sec >>> 24,
                    sec >>> 16,
                    sec >>> 8,
                    sec,
                ]);
            } else {
                // 32 bit nanoseconds, 64 bit seconds, negative values allowed.
                const ns = data.getMilliseconds() * 1000000;
                appendBytes([0xc7, 12, 0xff, ns >>> 24, ns >>> 16, ns >>> 8, ns]);
                appendInt64(sec);
            }
        }

        /**
         * Append a byte to the array.
         * @param data The byte to append.
         */
        function appendByte(data: number): void {
            if (array.length < length + 1) {
                let newLength = array.length * 2;
                while (newLength < length + 1) newLength *= 2;
                const newArray = new Uint8Array(newLength);
                newArray.set(array);
                array = newArray;
            }
            array[length] = data;
            length++;
        }

        /**
         * Append multiple bytes to the array.
         * @param data The bytes to append.
         */
        function appendBytes(data: number[] | Uint8Array | Uint8ClampedArray): void {
            if (array.length < length + data.length) {
                let newLength = array.length * 2;
                while (newLength < length + data.length) newLength *= 2;
                const newArray = new Uint8Array(newLength);
                newArray.set(array);
                array = newArray;
            }
            array.set(data, length);
            length += data.length;
        }

        /**
         * Append an int64 to the array.
         * @param data The number to append.
         */
        function appendInt64(data: number): void {
            // Split 64-bit number into two 32-bit numbers because JavaScript only regards 32 bits for bitwise operations.
            let hi, lo;
            if (data >= 0) {
                // Same as uint64
                hi = data / pow32;
                lo = data % pow32;
            } else {
                // Split absolute value to high and low, then NOT and ADD(1) to restore negativity.
                data++;
                hi = Math.abs(data) / pow32;
                lo = Math.abs(data) % pow32;
                hi = ~hi;
                lo = ~lo;
            }
            appendBytes([hi >>> 24, hi >>> 16, hi >>> 8, hi, lo >>> 24, lo >>> 16, lo >>> 8, lo]);
        }

        // Encodes a string to UTF-8 bytes.
        // Based on: https://gist.github.com/pascaldekloe/62546103a1576803dade9269ccf76330
        function encodeUtf8(str: string): Uint8Array {
            let ascii = true;
            const length = str.length;
            for (let x = 0; x < length; x++) {
                if (str.charCodeAt(x) > 127) {
                    ascii = false;
                    break;
                }
            }

            let i = 0;
            const bytes = new Uint8Array(str.length * (ascii ? 1 : 4));
            for (let ci = 0; ci !== length; ci++) {
                let c = str.charCodeAt(ci);
                if (c < 128) {
                    bytes[i++] = c;
                    continue;
                }
                if (c < 2048) {
                    bytes[i++] = (c >> 6) | 192;
                } else {
                    if (c > 0xd7ff && c < 0xdc00) {
                        if (++ci >= length) throw new Error(`UTF-8 encode: incomplete surrogate pair`);
                        const c2 = str.charCodeAt(ci);
                        if (c2 < 0xdc00 || c2 > 0xdfff)
                            throw new Error(`UTF-8 encode: second surrogate character 0x${c2.toString(16)} at index ${ci} out of range`);
                        c = 0x10000 + ((c & 0x03ff) << 10) + (c2 & 0x03ff);
                        bytes[i++] = (c >> 18) | 240;
                        bytes[i++] = ((c >> 12) & 63) | 128;
                    } else bytes[i++] = (c >> 12) | 224;
                    bytes[i++] = ((c >> 6) & 63) | 128;
                }
                bytes[i++] = (c & 63) | 128;
            }
            return ascii ? bytes : bytes.subarray(0, i);
        }
    }

    /**
     * Deserialize a MessagePack byte array.
     * @param data The data to deserialize.
     * @param options Deserialization options.
     * @returns The deserialized array.
     */
    public static deserialize(data: ArrayBuffer | Uint8Array, options?: { multiple?: boolean }): any[] {
        const array = data instanceof ArrayBuffer || !(data instanceof Uint8Array) ? new Uint8Array(data) : data;

        if (!array.length) throw new Error(`Invalid argument: The byte array to deserialize is empty.`);

        const pow32 = 0x100000000;
        let pos = 0;

        let deserializedData: any[];
        if (options && options.multiple) {
            deserializedData = [];
            while (pos < array.length) {
                deserializedData.push(read());
            }
        } else {
            deserializedData = read();
        }

        return deserializedData;

        /**
         * Read the next byte in the unserialized data byte array.
         * @returns Deserialized data from the next byte.
         */
        function read(): any {
            const byte = array[pos++];
            if (byte >= 0x00 && byte <= 0x7f) return byte; // positive fixint
            if (byte >= 0x80 && byte <= 0x8f) return readMap(byte - 0x80); // fixmap
            if (byte >= 0x90 && byte <= 0x9f) return readArray(byte - 0x90); // fixarray
            if (byte >= 0xa0 && byte <= 0xbf) return readString(byte - 0xa0); // fixstr
            if (byte === 0xc0) return null; // null
            if (byte === 0xc1) throw new Error(`Invalid byte code 0xc1 found.`); // never used
            if (byte === 0xc2) return false; // false
            if (byte === 0xc3) return true; // true
            if (byte === 0xc4) return readBinary(-1, 1); // bin 8
            if (byte === 0xc5) return readBinary(-1, 2); // bin 16
            if (byte === 0xc6) return readBinary(-1, 4); // bin 32
            if (byte === 0xc7) return readExtension(-1, 1); // ext 8
            if (byte === 0xc8) return readExtension(-1, 2); // ext 16
            if (byte === 0xc9) return readExtension(-1, 4); // ext 32
            if (byte === 0xca) return readFloat(4); // float 32
            if (byte === 0xcb) return readFloat(8); // float 64
            if (byte === 0xcc) return readUInt(1); // uint 8
            if (byte === 0xcd) return readUInt(2); // uint 16
            if (byte === 0xce) return readUInt(4); // uint 32
            if (byte === 0xcf) return readUInt(8); // uint 64
            if (byte === 0xd0) return readInt(1); // int 8
            if (byte === 0xd1) return readInt(2); // int 16
            if (byte === 0xd2) return readInt(4); // int 32
            if (byte === 0xd3) return readInt(8); // int 64
            if (byte === 0xd4) return readExtension(1); // fixext 1
            if (byte === 0xd5) return readExtension(2); // fixext 2
            if (byte === 0xd6) return readExtension(4); // fixext 4
            if (byte === 0xd7) return readExtension(8); // fixext 8
            if (byte === 0xd8) return readExtension(16); // fixext 16
            if (byte === 0xd9) return readString(-1, 1); // str 8
            if (byte === 0xda) return readString(-1, 2); // str 16
            if (byte === 0xdb) return readString(-1, 4); // str 32
            if (byte === 0xdc) return readArray(-1, 2); // array 16
            if (byte === 0xdd) return readArray(-1, 4); // array 32
            if (byte === 0xde) return readMap(-1, 2); // map 16
            if (byte === 0xdf) return readMap(-1, 4); // map 32
            if (byte >= 0xe0 && byte <= 0xff) return byte - 256; // negative fixint

            console.debug(`MessagePack Array:`, array);
            throw new Error(
                `Invalid byte value '${byte}' at index ${pos - 1} in the MessagePack binary data (length ${
                    array.length
                }): Expecting a range of 0 to 255. This is not a byte array.`,
            );
        }

        /**
         * Read an integer.
         * @param size The integer's size.
         * @returns The integer.
         */
        function readInt(size: number): number {
            let value = 0;
            let first = true;
            while (size-- > 0) {
                if (first) {
                    const byte = array[pos++];
                    value += byte & 0x7f;
                    if (byte & 0x80) {
                        value -= 0x80; // Treat most-significant bit as -2^i instead of 2^i
                    }
                    first = false;
                } else {
                    value *= 256;
                    value += array[pos++];
                }
            }
            return value;
        }

        /**
         * Read an unsigned integer.
         * @param size The integer's size.
         * @returns The integer.
         */
        function readUInt(size: number): number {
            let value = 0;
            while (size-- > 0) {
                value *= 256;
                value += array[pos++];
            }
            return value;
        }

        /**
         * Read a float.
         * @param size The float's size.
         * @returns The float.
         */
        function readFloat(size: number): number {
            const view = new DataView(array.buffer, pos + array.byteOffset, size);
            pos += size;
            if (size === 4) return view.getFloat32(0, false);
            if (size === 8) return view.getFloat64(0, false);
            else return 0;
        }

        /**
         * Read binary.
         * @param size The size of the binary value. -1 specifies to read an unsigned integer of lengthSize to determine the size.
         * @param lengthSize The size of an unsigned integer that specifies the length of the binary value.
         * @returns The binary value.
         */
        function readBinary(size: number, lengthSize?: number): Uint8Array {
            if (size < 0) {
                if (typeof lengthSize === `number`) size = readUInt(lengthSize);
                else throw new Error(`Argument mismatch`);
            }
            const data = array.subarray(pos, pos + size);
            pos += size;
            return data;
        }

        /**
         * Read a map.
         * @param size The size of the map. -1 specifies to read an unsigned integer of lengthSize to determine the size.
         * @param lengthSize The size of an unsigned integer that specifies the length of the map.
         * @returns The map.
         */
        function readMap(size: number, lengthSize?: number): Record<string, any> {
            if (size < 0) {
                if (typeof lengthSize === `number`) size = readUInt(lengthSize);
                else throw new Error(`Argument mismatch`);
            }
            const data: Record<string, any> = {};
            while (size-- > 0) {
                const key = read();
                data[key] = read();
            }
            return data;
        }

        /**
         * Read an array.
         * @param size The size of the array. -1 specifies to read an unsigned integer of lengthSize to determine the size.
         * @param lengthSize The size of an unsigned integer that specifies the length of the array.
         * @returns The array.
         */
        function readArray(size: number, lengthSize?: number): any[] {
            if (size < 0) {
                if (typeof lengthSize === `number`) size = readUInt(lengthSize);
                else throw new Error(`Argument mismatch`);
            }
            const data: any[] = [];
            while (size-- > 0) {
                data.push(read());
            }
            return data;
        }

        /**
         * Read a string.
         * @param size The size of the string. -1 specifies to read an unsigned integer of lengthSize to determine the size.
         * @param lengthSize The size of an unsigned integer that specifies the length of the string.
         * @returns The string.
         */
        function readString(size: number, lengthSize?: number): string {
            if (size < 0) {
                if (typeof lengthSize === `number`) size = readUInt(lengthSize);
                else throw new Error(`Argument mismatch`);
            }
            const start = pos;
            pos += size;
            return decodeUtf8(array, start, size);
        }

        /**
         * Read an extension.
         * @param size The size of the extension. -1 specifies to read an unsigned integer of lengthSize to determine the size.
         * @param lengthSize The size of an unsigned integer that specifies the length of the extension.
         * @returns The extension.
         */
        function readExtension(size: number, lengthSize?: number): Date | { type: number; data: Uint8Array } {
            if (size < 0) {
                if (typeof lengthSize === `number`) size = readUInt(lengthSize);
                else throw new Error(`Argument mismatch`);
            }
            const type = readUInt(1);
            const data = readBinary(size);
            switch (type) {
                case 255:
                    if (data.length === 4) {
                        const sec = ((data[0] << 24) >>> 0) + ((data[1] << 16) >>> 0) + ((data[2] << 8) >>> 0) + data[3];
                        return new Date(sec * 1000);
                    }
                    if (data.length === 8) {
                        const ns = ((data[0] << 22) >>> 0) + ((data[1] << 14) >>> 0) + ((data[2] << 6) >>> 0) + (data[3] >>> 2);
                        const sec =
                            (data[3] & 0x3) * pow32 + ((data[4] << 24) >>> 0) + ((data[5] << 16) >>> 0) + ((data[6] << 8) >>> 0) + data[7];
                        return new Date(sec * 1000 + ns / 1000000);
                    }
                    if (data.length === 12) {
                        const ns = ((data[0] << 24) >>> 0) + ((data[1] << 16) >>> 0) + ((data[2] << 8) >>> 0) + data[3];
                        pos -= 8;
                        const sec = readInt(8);
                        return new Date(sec * 1000 + ns / 1000000);
                    }
                    throw new Error(`Invalid data length for a date value.`);
            }
            return {
                type: type,
                data: data,
            };
        }

        /**
         * Decodes a string from UTF-8 bytes.
         * From https://gist.github.com/pascaldekloe/62546103a1576803dade9269ccf76330
         * @param bytes The bytes to decode.
         * @param start The index of the byte to start with.
         * @param length The number of bytes to decode.
         * @returns The decoded string.
         */
        function decodeUtf8(bytes: Uint8Array, start: number, length: number): string {
            let i = start,
                str = ``;
            length += start;
            while (i < length) {
                let c = bytes[i++];
                if (c > 127) {
                    if (c > 191 && c < 224) {
                        if (i >= length) throw new Error(`UTF-8 decode: incomplete 2-byte sequence`);
                        c = ((c & 31) << 6) | (bytes[i++] & 63);
                    } else if (c > 223 && c < 240) {
                        if (i + 1 >= length) throw new Error(`UTF-8 decode: incomplete 3-byte sequence`);
                        c = ((c & 15) << 12) | ((bytes[i++] & 63) << 6) | (bytes[i++] & 63);
                    } else if (c > 239 && c < 248) {
                        if (i + 2 >= length) throw new Error(`UTF-8 decode: incomplete 4-byte sequence`);
                        c = ((c & 7) << 18) | ((bytes[i++] & 63) << 12) | ((bytes[i++] & 63) << 6) | (bytes[i++] & 63);
                    } else throw new Error(`UTF-8 decode: unknown multibyte start 0x ${c.toString(16)} at index ${i - 1}`);
                }
                if (c <= 0xffff) str += String.fromCharCode(c);
                else if (c <= 0x10ffff) {
                    c -= 0x10000;
                    str += String.fromCharCode((c >> 10) | 0xd800);
                    str += String.fromCharCode((c & 0x3ff) | 0xdc00);
                } else throw new Error(`UTF-8 decode: code point 0x${c.toString(16)} exceeds UTF-16 reach`);
            }
            return str;
        }
    }
}
