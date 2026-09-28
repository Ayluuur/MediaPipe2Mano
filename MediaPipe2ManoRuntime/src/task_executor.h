#pragma once

#include <QFuture>
#include <QThreadPool>
#include <QtConcurrent/QtConcurrentRun>

#include <chrono>
#include <future>
#include <optional>
#include <type_traits>
#include <utility>

namespace m2m {

// 提供 std::future 风格接口；任务调度和线程生命周期由 QThreadPool 管理。
template <class T> class TaskFuture {
  public:
    TaskFuture() = default;
    explicit TaskFuture(QFuture<T> future) : future_(std::move(future)) {}

    bool valid() const noexcept {
        return future_.has_value();
    }

    template <class Rep, class Period>
    std::future_status wait_for(const std::chrono::duration<Rep, Period>&) const {
        return future_ && future_->isFinished() ? std::future_status::ready : std::future_status::timeout;
    }

    void wait() {
        if (future_)
            future_->waitForFinished();
    }

    T get() {
        QFuture<T> value = std::move(*future_);
        future_.reset();
        value.waitForFinished();
        return value.result();
    }

  private:
    std::optional<QFuture<T>> future_;
};

template <> class TaskFuture<void> {
  public:
    TaskFuture() = default;
    explicit TaskFuture(QFuture<void> future) : future_(std::move(future)) {}

    bool valid() const noexcept {
        return future_.has_value();
    }

    template <class Rep, class Period>
    std::future_status wait_for(const std::chrono::duration<Rep, Period>&) const {
        return future_ && future_->isFinished() ? std::future_status::ready : std::future_status::timeout;
    }

    void wait() {
        if (future_)
            future_->waitForFinished();
    }

    void get() {
        QFuture<void> value = std::move(*future_);
        future_.reset();
        value.waitForFinished();
    }

  private:
    std::optional<QFuture<void>> future_;
};

class TaskExecutor {
  public:
    explicit TaskExecutor(int maximumThreads = 1) {
        pool_.setMaxThreadCount(maximumThreads);
        pool_.setExpiryTimeout(-1);
    }

    ~TaskExecutor() {
        pool_.waitForDone();
    }

    TaskExecutor(const TaskExecutor&) = delete;
    TaskExecutor& operator=(const TaskExecutor&) = delete;

    template <class Function> auto submit(Function&& function) {
        using Result = std::invoke_result_t<std::decay_t<Function>>;
        return TaskFuture<Result>(QtConcurrent::run(&pool_, std::forward<Function>(function)));
    }

  private:
    QThreadPool pool_;
};

}
