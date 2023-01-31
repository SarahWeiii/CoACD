#pragma once

#include <algorithm>
#include <assert.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <ostream>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include <cstdlib>
#include <time.h>
#include <assert.h>
#include <algorithm>
#include <set>
#include <map>
#include <unordered_map>

namespace coacd
{
    class NullBuffer : public std::streambuf
    {
    public:
        int overflow(int c) { return c; }
    };

    std::ostream &logger(bool _cout_enabled, bool _logger_enabled, std::string const filename = "");
}