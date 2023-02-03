#pragma once
#include <exception>
#include <spdlog/spdlog.h>
#include <string_view>
namespace coacd
{
    namespace logger
    {

        std::shared_ptr<spdlog::logger> get();

        template <typename... Args>
        inline void debug(spdlog::string_view_t fmt, const Args &...args)
        {
            get()->debug(fmt, args...);
        };

        template <typename... Args>
        inline void info(spdlog::string_view_t fmt, const Args &...args)
        {
            get()->info(fmt, args...);
        };

        template <typename... Args>
        inline void warn(spdlog::string_view_t fmt, const Args &...args)
        {
            get()->warn(fmt, args...);
        };

        template <typename... Args>
        inline void error(spdlog::string_view_t fmt, const Args &...args)
        {
            get()->error(fmt, args...);
        };

        template <typename... Args>
        inline void critical(spdlog::string_view_t fmt, const Args &...args)
        {
            get()->critical(fmt, args...);
        };

    } // namespace logger
} // namespace coacd