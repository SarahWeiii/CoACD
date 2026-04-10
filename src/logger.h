#pragma once
#include <exception>
#ifndef DISABLE_SPDLOG
#include <spdlog/spdlog.h>

#if FMT_VERSION >= 80000
    #define COACD_RUNTIME_FMT(x) fmt::runtime(x)
#else
    #define COACD_RUNTIME_FMT(x) x
#endif
#else
#include <iostream>
#endif
#include <string_view>
namespace coacd
{
    namespace logger
    {

        #ifndef DISABLE_SPDLOG
        std::shared_ptr<spdlog::logger> get();
        #else
        template <typename Arg, typename... Args>
        void doPrint(std::ostream& out, Arg&& arg, Args&&... args)
        {
            out << std::forward<Arg>(arg);
            using expander = int[];
            (void)expander{0, (void(out << ',' << std::forward<Args>(args)), 0)...};
            out << std::endl;
        }
        #endif

        template <typename... Args>
        inline void debug(std::string_view fmt, const Args &...args)
        {
            #ifndef DISABLE_SPDLOG
            get()->debug(COACD_RUNTIME_FMT(fmt), args...);
            #else
            doPrint(std::cout, fmt, args...);
            #endif
        };

        template <typename... Args>
        inline void info(std::string_view fmt, const Args &...args)
        {
            #ifndef DISABLE_SPDLOG
            get()->info(COACD_RUNTIME_FMT(fmt), args...);
            #else
            doPrint(std::cout, fmt, args...);
            #endif
        };

        template <typename... Args>
        inline void warn(std::string_view fmt, const Args &...args)
        {
            #ifndef DISABLE_SPDLOG
            get()->warn(COACD_RUNTIME_FMT(fmt), args...);
            #else
            doPrint(std::cout, fmt, args...);
            #endif
        };

        template <typename... Args>
        inline void error(std::string_view fmt, const Args &...args)
        {
            #ifndef DISABLE_SPDLOG
            get()->error(COACD_RUNTIME_FMT(fmt), args...);
            #else
            doPrint(std::cout, fmt, args...);
            #endif
        };

        template <typename... Args>
        inline void critical(std::string_view fmt, const Args &...args)
        {
            #ifndef DISABLE_SPDLOG
            get()->critical(COACD_RUNTIME_FMT(fmt), args...);
            #else
            doPrint(std::cout, fmt, args...);
            #endif
        };

    } // namespace logger
} // namespace coacd