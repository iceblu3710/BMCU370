/**
 * @file LiteJSON.cpp
 * @brief Lightweight, error-tolerant JSON parser and serializer implementation.
 * 
 * DEVELOPMENT STATE: TESTING
 */

#include "LiteJSON.h"
#include <ctype.h>
#include <math.h>

namespace LiteJSON {

// Static null value for missing keys
LiteValue LiteObject::_nullValue;

// Static nested object pool for makeObject() support
LiteObject LiteValue::_nestedPool[MAX_NESTING];
int LiteValue::_nestedPoolIdx = 0;

// Static array pool for makeArray() support
LiteArray LiteValue::_arrayPool[MAX_NESTING];
int LiteValue::_arrayPoolIdx = 0;

// Fallback empty array for const access
static LiteArray _emptyArray;

// ============================================================================
// LiteValue Implementation
// ============================================================================

void LiteValue::set(const char* val) {
    if (!val) {
        setNull();
        return;
    }
    _type = ValueType::String;
    strncpy(_data.s, val, MAX_STRING_LEN - 1);
    _data.s[MAX_STRING_LEN - 1] = '\0';
}

// LiteValue methods implemented below

// LiteValue methods
LiteArray& LiteValue::makeArray() {
    _type = ValueType::Array;
    _arrayPoolIdx = (_arrayPoolIdx + 1) % MAX_NESTING;
    _arrayPool[_arrayPoolIdx] = LiteArray();
    _data.a = &_arrayPool[_arrayPoolIdx];
    return *_data.a;
}

LiteObject& LiteValue::makeObject() {
    _type = ValueType::Object;
    _nestedPoolIdx = (_nestedPoolIdx + 1) % MAX_NESTING;
    _nestedPool[_nestedPoolIdx] = LiteObject();
    _data.o = &_nestedPool[_nestedPoolIdx];
    return *_data.o;
}

static void serializeValueFlat(const LiteValue& val, char* buf, size_t size, size_t& offset) {
    auto write = [&](char c) { if (offset < size - 1) buf[offset++] = c; };
    auto writeStr = [&](const char* s) { while (s && *s && offset < size - 1) buf[offset++] = *s++; };
    
    switch (val.type()) {
        case ValueType::Null:   writeStr("null"); break;
        case ValueType::Bool:   writeStr(val.asBool() ? "true" : "false"); break;
        case ValueType::Int:    {
            char nbuf[16]; snprintf(nbuf, sizeof(nbuf), "%d", val.asInt());
            writeStr(nbuf); break;
        }
        case ValueType::Float:  {
            char nbuf[32]; float f = val.asFloat();
            if (!isfinite(f)) { writeStr("0.00"); break; }
            bool neg = f < 0; if (neg) f = -f;
            if (f > 2000000000.0f) {
                snprintf(nbuf, sizeof(nbuf), "%s%g", neg ? "-" : "", val.asFloat());
                writeStr(nbuf); break;
            }
            int i = (int)f; int d = (int)((f - i) * 100 + 0.5f);
            if (neg) write('-');
            snprintf(nbuf, sizeof(nbuf), "%d.%02d", i, d);
            writeStr(nbuf); break;
        }
        case ValueType::String: {
            write('"');
            const char* s = val.asString();
            while (s && *s && offset < size - 1) {
                if (*s == '"' || *s == '\\') write('\\');
                write(*s++);
            }
            write('"'); break;
        }
        case ValueType::Array: {
            const LiteArray* arr = val.getArrayPtr();
            write('[');
            if (arr) {
                for (int i = 0; i < arr->size(); i++) {
                    if (i > 0) write(',');
                    // Limited recursion for primitives in arrays is okay as depth is low
                    // But for objects it must call serializeValueFlat
                    val.getArray().serializeElementFlat(i, buf, size, offset);
                }
            }
            write(']'); break;
        }
        case ValueType::Object: {
            const LiteObject* obj = val.getObject();
            write('{');
            if (obj) {
                for (int i = 0; i < obj->size(); i++) {
                    if (i > 0) write(',');
                    write('"'); writeStr(obj->getKey(i)); writeStr("\":");
                    serializeValueFlat(obj->getValue(i), buf, size, offset);
                }
            }
            write('}'); break;
        }
    }
}

int LiteValue::serialize(char* buf, size_t size) const {
    size_t offset = 0;
    serializeValueFlat(*this, buf, size, offset);
    if (offset < size) buf[offset] = '\0';
    return (int)offset;
}

// ============================================================================
// LiteArray Implementation
// ============================================================================

bool LiteArray::add(int val) {
    if (_size >= MAX_ARRAY_SIZE) return false;
    _elements[_size].type = ValueType::Int;
    _elements[_size].val.i = val;
    _size++;
    return true;
}

bool LiteArray::add(float val) {
    if (_size >= MAX_ARRAY_SIZE) return false;
    _elements[_size].type = ValueType::Float;
    _elements[_size].val.f = val;
    _size++;
    return true;
}

bool LiteArray::add(bool val) {
    if (_size >= MAX_ARRAY_SIZE) return false;
    _elements[_size].type = ValueType::Bool;
    _elements[_size].val.b = val;
    _size++;
    return true;
}

bool LiteArray::add(const char* val) {
    if (_size >= MAX_ARRAY_SIZE) return false;
    _elements[_size].type = ValueType::String;
    if (val) {
        strncpy(_elements[_size].str, val, MAX_STRING_LEN - 1);
        _elements[_size].str[MAX_STRING_LEN - 1] = '\0';
    } else {
        _elements[_size].str[0] = '\0';
    }
    _size++;
    return true;
}

bool LiteArray::add(const LiteValue& v) {
    if (_size >= MAX_ARRAY_SIZE) return false;
    switch(v.type()) {
        case ValueType::Int:    return add(v.asInt());
        case ValueType::Float:  return add(v.asFloat());
        case ValueType::Bool:   return add(v.asBool());
        case ValueType::String: return add(v.asString());
        case ValueType::Object: 
            _elements[_size].type = ValueType::Object;
            _elements[_size].val.ptr = (void*)v.getObject();
            _size++;
            return true;
        case ValueType::Array:
            _elements[_size].type = ValueType::Array;
            _elements[_size].val.ptr = (void*)v.getArrayPtr();
            _size++;
            return true;
        default: return false;
    }
}

int LiteArray::getInt(int index) const {
    if (index < 0 || index >= _size) return 0;
    const Element& e = _elements[index];
    if (e.type == ValueType::Int) return e.val.i;
    if (e.type == ValueType::Float) return (int)e.val.f;
    return 0;
}

float LiteArray::getFloat(int index) const {
    if (index < 0 || index >= _size) return 0.0f;
    const Element& e = _elements[index];
    if (e.type == ValueType::Float) return e.val.f;
    if (e.type == ValueType::Int) return (float)e.val.i;
    return 0.0f;
}

bool LiteArray::getBool(int index) const {
    if (index < 0 || index >= _size) return false;
    return _elements[index].val.b;
}

const char* LiteArray::getString(int index) const {
    if (index < 0 || index >= _size) return "";
    return _elements[index].str;
}

void LiteArray::serializeElementFlat(int index, char* buf, size_t size, size_t& offset) const {
    if (index < 0 || index >= _size) return;
    const Element& e = _elements[index];
    auto write = [&](char c) { if (offset < size - 1) buf[offset++] = c; };
    auto writeStr = [&](const char* s) { while (s && *s && offset < size - 1) buf[offset++] = *s++; };
    
    switch (e.type) {
        case ValueType::Int: { char nbuf[16]; snprintf(nbuf, sizeof(nbuf), "%d", e.val.i); writeStr(nbuf); break; }
        case ValueType::Float: {
            char nbuf[32]; float f = e.val.f; if (!isfinite(f)) { writeStr("0.00"); break; }
            bool neg = f < 0; if(neg) f = -f; 
            if (f > 2000000000.0f) {
                snprintf(nbuf, sizeof(nbuf), "%s%g", neg ? "-" : "", e.val.f);
                writeStr(nbuf); break;
            }
            int i = (int)f; int d = (int)((f-i)*100+0.5f);
            if(neg) write('-'); 
            snprintf(nbuf, sizeof(nbuf), "%d.%02d", i, d); 
            writeStr(nbuf); 
            break; 
        }
        case ValueType::Bool: writeStr(e.val.b ? "true" : "false"); break;
        case ValueType::String: write('"'); writeStr(e.str); write('"'); break;
        case ValueType::Object: serializeValueFlat(LiteValue((LiteObject*)getElementPtr(index)), buf, size, offset); break;
        case ValueType::Array: serializeValueFlat(LiteValue((LiteArray*)getElementPtr(index)), buf, size, offset); break;
        default: break;
    }
}

int LiteArray::serialize(char* buf, size_t bufLen) const {
    size_t offset = 0;
    auto write = [&](char c) { if (offset < bufLen - 1) buf[offset++] = c; };
    write('[');
    for (int i = 0; i < _size; i++) {
        if (i > 0) write(',');
        serializeElementFlat(i, buf, bufLen, offset);
    }
    write(']');
    if (offset < bufLen) buf[offset] = '\0';
    return (int)offset;
}

// ============================================================================
// LiteObject Implementation
// ============================================================================

LiteValue& LiteObject::operator[](const char* key) {
    // Search for existing key
    for (int i = 0; i < _count; i++) {
        if (strcmp(_pairs[i].key, key) == 0) {
            return _pairs[i].value;
        }
    }
    
    // Create new key if space available
    if (_count < MAX_KEYS) {
        strncpy(_pairs[_count].key, key, 31);
        _pairs[_count].key[31] = '\0';
        _pairs[_count].value = LiteValue();
        return _pairs[_count++].value;
    }
    
    // Overflow - return static null
    _valid = false;
    return _nullValue;
}

const LiteValue& LiteObject::operator[](const char* key) const {
    for (int i = 0; i < _count; i++) {
        if (strcmp(_pairs[i].key, key) == 0) {
            return _pairs[i].value;
        }
    }
    return _nullValue;
}

bool LiteObject::containsKey(const char* key) const {
    for (int i = 0; i < _count; i++) {
        if (strcmp(_pairs[i].key, key) == 0) {
            return true;
        }
    }
    return false;
}

int LiteObject::serialize(char* buf, size_t bufLen) const {
    size_t offset = 0;
    auto write = [&](char c) { if (offset < (size_t)bufLen - 1) buf[offset++] = c; };
    auto writeStr = [&](const char* s) { while (s && *s && offset < (size_t)bufLen - 1) buf[offset++] = *s++; };
    
    write('{');
    for (int i = 0; i < _count; i++) {
        if (i > 0) write(',');
        write('"'); writeStr(_pairs[i].key); writeStr("\":");
        size_t n = _pairs[i].value.serialize(buf + offset, bufLen - offset);
        offset += n;
    }
    write('}');
    if (offset < (size_t)bufLen) buf[offset] = '\0';
    return (int)offset;
}

// ============================================================================
// LiteDoc Implementation
// ============================================================================

void LiteDoc::clear() {
    _root = LiteObject();
    _error = ParseError::None;
}

const char* LiteDoc::skipWhitespace(const char* p) {
    while (p && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) {
        p++;
    }
    return p;
}

const char* LiteDoc::parseString(const char* p, char* out, size_t maxLen) {
    if (!p || *p != '"') {
        _error = ParseError::InvalidSyntax;
        return nullptr;
    }
    p++;
    
    size_t i = 0;
    while (*p && *p != '"') {
        if (*p == '\\') {
            p++;
            if (!*p) { _error = ParseError::UnterminatedString; return nullptr; }
            if (i < maxLen - 1) {
                switch(*p) {
                    case 'n': out[i++] = '\n'; break;
                    case 'r': out[i++] = '\r'; break;
                    case 't': out[i++] = '\t'; break;
                    case '"': out[i++] = '"'; break;
                    case '\\': out[i++] = '\\'; break;
                    default: out[i++] = *p; break;
                }
            }
        } else {
            if (i < maxLen - 1) out[i++] = *p;
        }
        p++;
    }
    
    if (*p == '"') {
        if (i < maxLen) out[i] = '\0';
        p++;
    } else {
        _error = ParseError::UnterminatedString;
        return nullptr;
    }
    
    return p;
}

const char* LiteDoc::parseNumber(const char* p, LiteValue& val) {
    if (!p || !*p) return nullptr;
    
    char numBuf[32];
    int i = 0;
    bool isFloat = false;
    
    // Handle leading sign
    if (*p == '-' || *p == '+') numBuf[i++] = *p++;
    
    while (*p && (isdigit(*p) || *p == '.' || *p == 'e' || *p == 'E' || *p == '-' || *p == '+')) {
        if (i < 31) {
            if (*p == '.' || *p == 'e' || *p == 'E') isFloat = true;
            numBuf[i++] = *p;
        }
        p++;
    }
    numBuf[i] = '\0';
    
    if (i == 0 || (i == 1 && (numBuf[0] == '-' || numBuf[0] == '+'))) {
        _error = ParseError::InvalidNumber;
        return nullptr;
    }
    
    // Safe manual parsing to avoid atof/atoi crashes on extreme values
    if (isFloat || i > 9) {
        float result = 0.0f;
        float fraction = 1.0f;
        int d = 0;
        int sign = 1;
        bool inFraction = false;
        
        const char* n = numBuf;
        if (*n == '-') { sign = -1; n++; }
        else if (*n == '+') { n++; }
        
        while (*n) {
            if (*n == '.') {
                inFraction = true;
            } else if (*n >= '0' && *n <= '9') {
                if (!inFraction) {
                    result = result * 10.0f + (*n - '0');
                } else {
                    fraction *= 0.1f;
                    result += (*n - '0') * fraction;
                }
            } else if (*n == 'e' || *n == 'E') {
                 // Simple exponent handling ignoring crashes
                 break; 
            }
            n++;
        }
        val.set(result * sign);
    }
    else val.set((int)atoi(numBuf));
    
    return p;
}

const char* LiteDoc::parseArray(const char* p, LiteArray& arr, int depth) {
    if (!p || depth > MAX_NESTING) { _error = ParseError::NestingTooDeep; return nullptr; }
    if (*p != '[') return nullptr;
    p++;
    
    while (p && *p) {
        p = skipWhitespace(p);
        if (*p == ']') break;
        
        LiteValue elemVal;
        p = parseValue(p, elemVal, depth);
        if (!p) break;
        
        arr.add(elemVal); // Uses operator assignment which is safe
        
        p = skipWhitespace(p);
        if (*p == ',') {
            p++;
            // STRICT: check if trailing comma or missing value
            p = skipWhitespace(p);
            if (*p == ']') { _error = ParseError::InvalidSyntax; return nullptr; }
        } else if (*p != ']') {
            // STRICT: comma required
            _error = ParseError::InvalidSyntax; return nullptr;
        }
    }
    
    if (p && *p == ']') p++;
    else _error = ParseError::InvalidSyntax;
    
    return p;
}

const char* LiteDoc::parseValue(const char* p, LiteValue& val, int depth) {
    if (!p || depth > MAX_NESTING) return nullptr;
    
    p = skipWhitespace(p);
    if (!*p) return p;
    
    // Null
    if (strncmp(p, "null", 4) == 0) {
        val.setNull();
        return p + 4;
    }
    
    // Boolean true
    if (strncmp(p, "true", 4) == 0) {
        val.set(true);
        return p + 4;
    }
    
    // Boolean false  
    if (strncmp(p, "false", 5) == 0) {
        val.set(false);
        return p + 5;
    }
    
    // String
    if (*p == '"') {
        char strBuf[MAX_STRING_LEN];
        p = parseString(p, strBuf, MAX_STRING_LEN);
        val.set(strBuf);
        return p;
    }
    
    // Number
    if (*p == '-' || isdigit(*p)) {
        return parseNumber(p, val);
    }
    
    // Array
    if (*p == '[') {
        val.setArray();
        return parseArray(p, val.getArray(), depth);
    }
    
    // Nested object - recursively parse
    if (*p == '{') {
        LiteObject& obj = val.makeObject();
        return parseObject(p, obj, depth + 1);
    }
    
    // Unknown - error
    _error = ParseError::InvalidSyntax;
    return nullptr;
}

const char* LiteDoc::parseObject(const char* p, LiteObject& obj, int depth) {
    if (!p || depth > MAX_NESTING) { _error = ParseError::NestingTooDeep; return nullptr; }
    
    p = skipWhitespace(p);
    if (*p != '{') return nullptr;
    p++;
    
    while (p && *p) {
        p = skipWhitespace(p);
        if (*p == '}') break;
        
        // STRICT: Key MUST be quoted
        if (*p != '"') { _error = ParseError::InvalidSyntax; return nullptr; }
        // Parse key
        char keyBuf[16]; keyBuf[0] = '\0';
        if (p && *p == '"') {
            p = parseString(p, keyBuf, 16);
        } else {
            // STRICT: Key MUST be quoted
            _error = ParseError::InvalidSyntax;
            return nullptr;
        }
        if (!p) break; // Check if parseString failed
        
        p = skipWhitespace(p);
        if (*p != ':') { _error = ParseError::InvalidSyntax; return nullptr; }
        p++;
        p = skipWhitespace(p);
        
        p = parseValue(p, obj[keyBuf], depth + 1);
        if (!p) break;
        
        p = skipWhitespace(p);
        if (*p == ',') {
            p++;
            // check for trailing comma
            p = skipWhitespace(p);
            if (*p == '}') { _error = ParseError::InvalidSyntax; return nullptr; }
        } else if (*p != '}') {
            _error = ParseError::InvalidSyntax; return nullptr;
        }
    }
    
    if (p && *p == '}') p++;
    else _error = ParseError::InvalidSyntax;
    
    return p;
}

ParseError LiteDoc::parse(const char* json) {
    clear();
    
    if (!json || !*json) {
        _error = ParseError::EmptyInput;
        return _error;
    }
    
    const char* p = skipWhitespace(json);
    
    if (*p != '{') {
        _error = ParseError::InvalidSyntax;
        return _error;
    }
    
    const char* end = parseObject(p, _root, 0);
    
    if (!end) {
        _error = ParseError::InvalidSyntax;
    } else if (!_root.isValid()) {
        _error = ParseError::TooManyKeys;
    }
    
    return _error;
}

size_t LiteDoc::serialize(char* buf, size_t bufLen) const {
    return _root.serialize(buf, bufLen);
}

LiteValue& LiteDoc::operator[](const char* key) {
    return _root[key];
}

const LiteValue& LiteDoc::operator[](const char* key) const {
    return _root[key];
}

const char* LiteDoc::errorString() const {
    switch (_error) {
        case ParseError::None: return "No error";
        case ParseError::EmptyInput: return "Empty input";
        case ParseError::InvalidSyntax: return "Invalid syntax";
        case ParseError::UnterminatedString: return "Unterminated string";
        case ParseError::InvalidNumber: return "Invalid number";
        case ParseError::NestingTooDeep: return "Nesting too deep";
        case ParseError::TooManyKeys: return "Too many keys";
        case ParseError::BufferOverflow: return "Buffer overflow";
        default: return "Unknown error";
    }
}

const char* DeserializationResult::c_str() const {
    switch (error) {
        case ParseError::None: return "No error";
        case ParseError::EmptyInput: return "EmptyInput";
        case ParseError::InvalidSyntax: return "InvalidSyntax";
        case ParseError::UnterminatedString: return "UnterminatedString";
        case ParseError::InvalidNumber: return "InvalidNumber";
        case ParseError::NestingTooDeep: return "NestingTooDeep";
        case ParseError::TooManyKeys: return "TooManyKeys";
        case ParseError::BufferOverflow: return "BufferOverflow";
        default: return "UnknownError";
    }
}

} // namespace LiteJSON
