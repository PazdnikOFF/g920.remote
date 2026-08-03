#include "g920/version.h"

#ifndef G920_ROLE
#define G920_ROLE G920_ROLE_UNKNOWN
#endif

g920_version_t g920_firmware_version(void)
{
    g920_version_t v = { G920_FW_VERSION_MAJOR, G920_FW_VERSION_MINOR,
                         G920_FW_VERSION_PATCH };
    return v;
}

g920_role_t g920_build_role(void)
{
    return (g920_role_t)G920_ROLE;
}

const char *g920_role_name(g920_role_t role)
{
    switch (role) {
    case G920_ROLE_TX:
        return "TX";
    case G920_ROLE_RX:
        return "RX";
    case G920_ROLE_HOST_TEST:
        return "TEST";
    case G920_ROLE_UNKNOWN:
    default:
        return "??";
    }
}

uint32_t g920_version_pack(g920_version_t version)
{
    return ((uint32_t)version.major << 16) | ((uint32_t)version.minor << 8)
           | (uint32_t)version.patch;
}

g920_version_t g920_version_unpack(uint32_t packed)
{
    g920_version_t v = { (uint8_t)((packed >> 16) & 0xFFu),
                         (uint8_t)((packed >> 8) & 0xFFu),
                         (uint8_t)(packed & 0xFFu) };
    return v;
}

int g920_version_compare(g920_version_t a, g920_version_t b)
{
    if (a.major != b.major) {
        return a.major < b.major ? -1 : 1;
    }
    if (a.minor != b.minor) {
        return a.minor < b.minor ? -1 : 1;
    }
    if (a.patch != b.patch) {
        return a.patch < b.patch ? -1 : 1;
    }
    return 0;
}

bool g920_link_proto_compatible(uint32_t local, uint32_t remote)
{
    if (local == 0u || remote == 0u) {
        return false;
    }
    return local == remote;
}

int g920_version_format(char *buf, size_t size, g920_version_t version)
{
    /* Своя печать вместо snprintf: обрезанный вывод здесь недопустим, а
     * поведение snprintf при нехватке места пришлось бы разбирать по
     * возвращаемому значению на каждой стороне. */
    char tmp[G920_VERSION_STR_MAX];
    size_t len = 0;
    const uint8_t parts[3] = { version.major, version.minor, version.patch };

    if (buf == NULL) {
        return -1;
    }

    for (int i = 0; i < 3; i++) {
        uint8_t value = parts[i];
        char digits[3];
        int ndigits = 0;

        if (i > 0) {
            tmp[len++] = '.';
        }
        do {
            digits[ndigits++] = (char)('0' + (value % 10u));
            value = (uint8_t)(value / 10u);
        } while (value != 0u);
        while (ndigits > 0) {
            tmp[len++] = digits[--ndigits];
        }
    }

    if (size < len + 1u) {
        return -1;
    }
    for (size_t i = 0; i < len; i++) {
        buf[i] = tmp[i];
    }
    buf[len] = '\0';
    return (int)len;
}
