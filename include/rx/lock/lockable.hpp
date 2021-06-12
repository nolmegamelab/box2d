// lockable.hpp
#ifndef RX_LOCK_LOCKABLE_HPP_
#define RX_LOCK_LOCKABLE_HPP_

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif  // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <cassert>
#include <atomic>
#include <shared_mutex>
#include <string_view>

namespace rx {

class lockable : protected std::shared_mutex
{
public: 
  static constexpr int retry_sleep_count = 1000;

public:
  lockable()
    : name_(""),
      mode_latch_(0)
  {}
  explicit lockable(const char* name)
    : name_(name),
      mode_latch_(0)
  {}
  lockable(lockable&& other) noexcept
    : name_(""),
      mode_latch_(0)
  {
    swap(other);
  }

  const std::string_view& name() const
  {
    return name_;
  }

  void lock() noexcept
  {
    int bv = 0;

    do
    {
      wait_latch();

      std::shared_mutex::lock(); 

      bv = mode_latch_;

      if (bv > 0) // lock 호출 동안에 래치가 잡혀있다면 
      {
        // 래치가 잡혀 있으면 나는 포기
        std::shared_mutex::unlock();
      }
      else
      {
        break; // 값을 다시 조회하기 전에 같은 값으로 처리되어야 함
      }
    } while (bv > 0);
  }

  void unlock() noexcept 
  {
    std::shared_mutex::unlock();
  }

  void lock_shared() noexcept 
  {
    int bv = 0;

    do
    {
      wait_latch();

      std::shared_mutex::lock_shared();

      bv = mode_latch_;

      if (bv > 0)
      {
        // 래치가 잡혀 있으면 나는 포기
        std::shared_mutex::unlock_shared();
      }
      else
      {
        break; // 값을 다시 조회하기 전에 같은 값으로 처리되어야 함
      }
    } while (bv > 0);
  }

  void unlock_shared() noexcept 
  {
    std::shared_mutex::unlock_shared();
  }

  void upgrade() noexcept
  {
    assert(mode_latch_ >= 0);

    ++mode_latch_;                          

    std::shared_mutex::unlock_shared();
    std::shared_mutex::lock();              // 여기서 대기하는 것으로 upgrade 간 경쟁 해결

    --mode_latch_;
  }

  void downgrade() noexcept
  {
    assert(mode_latch_ >= 0);

    ++mode_latch_;

    std::shared_mutex::unlock();
    std::shared_mutex::lock_shared();

    --mode_latch_;
  }

private: 

  void wait_latch()
  {
      int try_count = 0;

      while (mode_latch_)
      {
        ++try_count;

        if (try_count >= retry_sleep_count)
        {
          sleep(1);
          try_count = 0;
        }
      }
  }

  void sleep(int ms)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  }

private:
  void swap(lockable& other)
  {
    std::swap(name_, other.name_);
    // atomic 교환은 아니지만 목적이 vector 컨테이너에 넣는 것이 목적이므로 감수
    mode_latch_.exchange(other.mode_latch_);
  }

  std::string_view name_;
  std::atomic<int> mode_latch_;
};

} // namsspace rx

#endif  // RX_LOCK_LOCKABLE_HPP_

// lockable.hpp
