//
#pragma once

#include <pico/critical_section.h>

class Mutex
{
public:
    Mutex() { critical_section_init(&cs_); }
    void lock() { critical_section_enter_blocking(&cs_); }
    void unlock() { critical_section_exit(&cs_); }
    critical_section_t *native_handle() { return &cs_; }

private:
    critical_section_t cs_;
};