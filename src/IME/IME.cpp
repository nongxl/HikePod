#include "IME.h"

#define _log(...) Serial.printf(__VA_ARGS__)

static const uint8_t IME_MAGIC[4] = {'I', 'M', 'E', '4'};

extern const uint8_t _binary_src_IME_ime_table_bin_start[];
extern const uint8_t _binary_src_IME_ime_table_bin_end[];

// Parse the IME3 header + prefix index from the start of the dictionary blob.
// `total` is the whole (padded) slot size; `_count` bounds the real records.
bool IME::parseHeader(const uint8_t *hdrIndex, size_t total)
{
    if (memcmp(hdrIndex, IME_MAGIC, 4) != 0)
    {
        _log("[IME] bad dictionary magic\n");
        return false;
    }

    _scheme = (Scheme)hdrIndex[4];
    _codeLen = hdrIndex[5];
    if (_codeLen < 1 || _codeLen > MAX_CODE_LEN)
    {
        _log("[IME] bad codeLen %d\n", _codeLen);
        return false;
    }
    _recordSize = _codeLen + POOLREF_SIZE;

    // Max letters the user may type per scheme.
    switch (_scheme)
    {
    case PINYIN:    _maxCode = 6; break;
    case SHUANGPIN: _maxCode = 2; break;
    case WUBI:
    default:        _maxCode = 4; break;
    }

    _count = (uint32_t)hdrIndex[8] | ((uint32_t)hdrIndex[9] << 8) |
             ((uint32_t)hdrIndex[10] << 16) | ((uint32_t)hdrIndex[11] << 24);

    _poolBytes = (uint32_t)hdrIndex[12] | ((uint32_t)hdrIndex[13] << 8) |
                 ((uint32_t)hdrIndex[14] << 16) | ((uint32_t)hdrIndex[15] << 24);

    _recordBase = HEADER_SIZE + (size_t)INDEX_ENTRIES * 4;
    _poolBase = _recordBase + (size_t)_count * _recordSize;

    size_t need = _poolBase + _poolBytes;
    if (_count == 0 || need > total)
    {
        _log("[IME] dictionary size mismatch: need %u, have %u\n",
             (unsigned)need, (unsigned)total);
        return false;
    }

    // Copy the prefix index into RAM (little-endian uint32s, 2.7 KB).
    _index.resize(INDEX_ENTRIES);
    for (int k = 0; k < INDEX_ENTRIES; k++)
    {
        const uint8_t *p = hdrIndex + HEADER_SIZE + (size_t)k * 4;
        _index[k] = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                    ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    }
    return true;
}

bool IME::begin()
{
    if (_loaded)
        return true;

    // Dictionary is memory-mapped in flash - no SD, no file handle.
    _blob = _binary_src_IME_ime_table_bin_start;
    _blobSize = (size_t)(_binary_src_IME_ime_table_bin_end - _binary_src_IME_ime_table_bin_start);
    if (_blobSize < HEADER_SIZE || !parseHeader(_blob, _blobSize))
    {
        _blob = nullptr;
        return false;
    }
    _loaded = true;
    static const char *NAMES[] = {"Wubi", "Pinyin", "Shuangpin"};
    _log("[IME] ready: %s, %u records, codeLen %d (embedded in flash)\n",
         NAMES[_scheme <= SHUANGPIN ? _scheme : 0], (unsigned)_count, _codeLen);
    return true;
}

// Read the code of record `i` (NUL-terminated) from the flash blob.
bool IME::readCode(uint32_t i, char out[MAX_CODE_LEN + 1])
{
    const uint8_t *rec = _blob + _recordBase + (size_t)i * _recordSize;
    int n = 0;
    for (; n < _codeLen && rec[n]; n++)
        out[n] = (char)rec[n];
    out[n] = '\0';
    return true;
}

// Read record `i`'s hanzi/phrase from the string pool. The record stores a
// uint24 LE offset into the pool + a 1-byte length; the text is raw UTF-8.
bool IME::readWord(uint32_t i, String &out)
{
    const uint8_t *ref = _blob + _recordBase + (size_t)i * _recordSize + _codeLen;
    size_t off = (size_t)ref[0] | ((size_t)ref[1] << 8) | ((size_t)ref[2] << 16);
    size_t len = ref[3];
    if (len == 0 || off + len > _poolBytes)
        return false;
    const char *p = (const char *)(_blob + _poolBase + off);
    out = "";
    out.reserve(len);
    for (size_t k = 0; k < len; k++)
        out += p[k];
    return true;
}

void IME::setActive(bool on)
{
    _active = on;
    reset();
}

void IME::reset()
{
    _code = "";
    _all.clear();
    _page.clear();
    _pageStart = 0;
}

// Narrow to the record window that can match the first one/two typed letters,
// using the RAM prefix index. With no index (legacy WUB1) this is [0,_count).
void IME::searchWindow(const char *code, int len, uint32_t &lo, uint32_t &hi)
{
    lo = 0;
    hi = _count;
    if (_index.empty() || len < 1)
        return;

    int c0 = code[0] - 'a';
    if (c0 < 0 || c0 >= 26)
        return; // out of a-z: keep full range (shouldn't happen)

    if (len == 1)
    {
        // all codes starting with c0: buckets [c0*26 .. (c0+1)*26)
        lo = _index[c0 * 26];
        hi = _index[(c0 + 1) * 26];
        return;
    }

    int c1 = code[1] - 'a';
    if (c1 < 0 || c1 >= 26)
        return;
    int k = c0 * 26 + c1;
    lo = _index[k];
    hi = _index[k + 1]; // sentinel guarantees k+1 <= INDEX_ENTRIES-1
}

void IME::lookup()
{
    _all.clear();
    _pageStart = 0;

    if (!_loaded || _code.length() == 0)
    {
        buildPage();
        return;
    }

    const char *q = _code.c_str();
    int qlen = _code.length();

    // Jump straight to the small record window for this prefix, then binary
    // search for the first record whose code >= the typed prefix within it.
    uint32_t lo, hi;
    searchWindow(q, qlen, lo, hi);
    uint32_t scanEnd = hi; // remember the window end for the forward scan
    while (lo < hi)
    {
        uint32_t mid = lo + (hi - lo) / 2;
        char code[7];  // MAX_CODE_LEN + 1
        if (!readCode(mid, code))
            break;
        if (strncmp(code, q, qlen) < 0)
            lo = mid + 1;
        else
            hi = mid;
    }

    // Walk forward collecting every record whose code starts with the prefix,
    // de-duplicating hanzi (the table can list the same char under several codes).
    // scanEnd bounds the walk to the prefix window from the index.
    for (uint32_t i = lo; i < scanEnd; i++)
    {
        char code[7];  // MAX_CODE_LEN + 1
        if (!readCode(i, code))
            break;
        if (strncmp(code, q, qlen) != 0)
            break;

        String h;
        if (!readWord(i, h))
            break;

        bool dup = false;
        for (auto &e : _all)
            if (e == h)
            {
                dup = true;
                break;
            }
        if (!dup)
            _all.push_back(h);

        // cap to keep paging snappy
        if (_all.size() >= 60)
            break;
    }

    buildPage();
}

void IME::buildPage()
{
    _page.clear();
    for (int i = _pageStart; i < (int)_all.size() && (int)_page.size() < PAGE_SIZE; i++)
        _page.push_back(_all[i]);
}

bool IME::commit(int idx, String &out)
{
    if (idx < 0 || idx >= (int)_page.size())
        return false;
    out = _page[idx];
    reset();
    return true;
}

bool IME::handleKey(int key, String &out)
{
    if (!_active)
        return false;

    // letters a-z (and A-Z) build the code (cap depends on the scheme)
    if ((key >= 'a' && key <= 'z') || (key >= 'A' && key <= 'Z'))
    {
        if ((int)_code.length() < _maxCode)
        {
            _code += (char)tolower(key);
            lookup();
        }
        return true;
    }

    // nothing in progress: only the explicit control keys below are ours,
    // everything else (including normal punctuation) goes to the editor.
    if (_code.length() == 0)
        return false;

    // digits select a candidate on the current page (1..9)
    if (key >= '1' && key <= '9')
    {
        if (commit(key - '1', out))
            return true;
        return true; // consumed even if out of range
    }

    // space commits the first candidate
    if (key == ' ')
    {
        if (_page.size() > 0)
            commit(0, out);
        else
            reset(); // no match - just drop the bad code
        return true;
    }

    // enter commits the typed letters as-is? No - clear, treat enter as commit#1
    if (key == '\n')
    {
        if (_page.size() > 0)
            commit(0, out);
        else
            reset();
        return true;
    }

    // backspace removes the last code letter
    if (key == '\b')
    {
        _code.remove(_code.length() - 1);
        if (_code.length() == 0)
            reset();
        else
            lookup();
        return true;
    }

    // ESC cancels the whole composition
    if (key == 27)
    {
        reset();
        return true;
    }

    // paging: '-' / ';' / ',' previous page, '=' / '\'' / '.' next page.
    if (key == '-' || key == ';' || key == ',')
    {
        if (_pageStart - PAGE_SIZE >= 0)
        {
            _pageStart -= PAGE_SIZE;
            buildPage();
        }
        return true;
    }
    if (key == '=' || key == '\'' || key == '.')
    {
        if (_pageStart + PAGE_SIZE < (int)_all.size())
        {
            _pageStart += PAGE_SIZE;
            buildPage();
        }
        return true;
    }

    // Any other key while composing (e.g. punctuation): commit the best
    // candidate and consume the key.
    if (_page.size() > 0)
    {
        commit(0, out);
        return true;
    }

    // Unmatched code + unknown key: drop the dead code, consume the key.
    reset();
    return true;
}
