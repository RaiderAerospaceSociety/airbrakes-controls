// ===== RTOS Mutex Helpers =====
// Brief: Null-safe, one-line mutex wrappers and RAII guards for FreeRTOS.
// Example usage:
/*
?   ENTER/EXIT macros (replaces check/give or check/take pairs with a one-liner)
ENTER_CRITICAL(g_spi_mutex);
    ... critical section ...
EXIT_CRITICAL(g_spi_mutex);

?   ENTER with timeout (10 ms) and check
if (!ENTER_CRITICAL_MS(g_i2c_mutex, 10)) {
    ... handle lock timeout ...
}
... critical section ...
EXIT_CRITICAL(g_i2c_mutex);

?   WITH_MUTEX block (auto-unlocks at closing brackets)
WITH_MUTEX(g_i2c_mutex) {
    ... critical section ...
}

?   RAII guard (auto-release at scope end)
{
    rtos::MutexLock guard{g_spi_mutex};  // waits forever by default
    if (!guard.locked()) { ... handle failure ... }
    ... critical section ...
} // unlocked automatically
*/

#pragma once

//* ===== Includes =====
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

//* ===== RTOS Mutex API Extension =====
namespace rtos
{
    // Create a standard RTOS mutex; returns null on failure
    inline SemaphoreHandle_t mutex_create() { return xSemaphoreCreateMutex(); }

    // Check if a mutex handle is valid (non-null); returns true if valid
    inline bool mutex_valid(SemaphoreHandle_t h) { return h != nullptr; }

    // Take a mutex with timeout, null-safe; default: portMAX_DELAY
    inline bool mutex_take(SemaphoreHandle_t h, TickType_t ticks = portMAX_DELAY)
    {
        if (!h)
            return true; // null-safe no-op success
        return xSemaphoreTake(h, ticks) == pdTRUE;
    }

    // Give a mutex null-safe; returns true on success
    inline bool mutex_give(SemaphoreHandle_t h)
    {
        if (!h)
            return true; // null-safe no-op success
        return xSemaphoreGive(h) == pdTRUE;
    }

    // RAII guard that locks a mutex on construction and unlocks on destruction (goes out of scope)
    class MutexLock
    {
    public:
        /** Construct and attempt to lock. */
        explicit MutexLock(SemaphoreHandle_t h, TickType_t ticks = portMAX_DELAY)
            : handle_(h)
        {
            locked_ = mutex_take(handle_, ticks);
        }
        /** Unlock (if locked) on destruction. */
        ~MutexLock()
        {
            if (locked_)
                (void)mutex_give(handle_);
        }

        /** Whether the mutex is currently held by this guard. */
        bool locked() const { return locked_; }

        /** Manually unlock early (idempotent). */
        void unlock()
        {
            if (locked_)
            {
                (void)mutex_give(handle_);
                locked_ = false;
            }
        }

        // Non-copyable
        MutexLock(const MutexLock &) = delete;
        MutexLock &operator=(const MutexLock &) = delete;

        // Movable (transfer ownership of the lock state)
        MutexLock(MutexLock &&other) noexcept : handle_(other.handle_), locked_(other.locked_)
        {
            other.locked_ = false;
        }
        MutexLock &operator=(MutexLock &&other) noexcept
        {
            if (this != &other)
            {
                if (locked_)
                    (void)mutex_give(handle_);
                handle_ = other.handle_;
                locked_ = other.locked_;
                other.locked_ = false;
            }
            return *this;
        }

    private:
        SemaphoreHandle_t handle_{nullptr};
        bool locked_{false};
    };

}

//* ===== Convenience Macros =====
// These macros are expression-like and return bool (lock success). Using them
// as a stand-alone statement is fine; the return value is simply ignored.

/** \brief Take a mutex forever (null-safe). */
#define ENTER_CRITICAL(mtx_) (rtos::mutex_take((mtx_), portMAX_DELAY))
/** \brief Take a mutex with a millisecond timeout (null-safe). */
#define ENTER_CRITICAL_MS(mtx_, ms) (rtos::mutex_take((mtx_), pdMS_TO_TICKS((ms))))
/** \brief Take a mutex with a tick timeout (null-safe). */
#define ENTER_CRITICAL_TICKS(mtx_, ticks) (rtos::mutex_take((mtx_), (ticks)))
/** \brief Try to take a mutex without blocking (null-safe). */
#define TRY_ENTER_CRITICAL(mtx_) (rtos::mutex_take((mtx_), 0))
/** \brief Give a mutex (null-safe). */
#define EXIT_CRITICAL(mtx_) (rtos::mutex_give((mtx_)))

// Uses brackets to take/give mutex (critical code in brackets); auto-unlocks at closing bracket
#define WITH_MUTEX(mtx_) for (rtos::MutexLock _rtos_mx_guard{(mtx_)}; _rtos_mx_guard.locked(); _rtos_mx_guard.unlock())