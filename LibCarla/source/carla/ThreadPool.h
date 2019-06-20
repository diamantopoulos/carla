// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/MoveHandler.h"
#include "carla/NonCopyable.h"
#include "carla/ThreadGroup.h"
#include "carla/Time.h"

#include <boost/asio/io_context.hpp>
#include <boost/asio/strand.hpp>

#include <future>
#include <thread>
#include <type_traits>

namespace carla {

  // ===========================================================================
  // -- Implementation details -------------------------------------------------
  // ===========================================================================

namespace detail {

  template <typename ResultT, typename ExecutorT, typename FunctorT>
  inline static std::future<ResultT> PostPackagedTask(ExecutorT &exec, FunctorT &&functor) {
    auto task = std::packaged_task<ResultT()>(std::forward<FunctorT>(functor));
    auto future = task.get_future();
    exec.post(carla::MoveHandler(task));
    return future;
  }

  template <typename ExecutorT, typename FunctorT>
  inline static void PostRawTask(ExecutorT &exec, FunctorT &&functor) {
    static_assert(
        noexcept(functor()),
        "functor should be marked as noexcept, please make sure it does not throw.");
    exec.post(std::forward<FunctorT>(functor));
  }

} // namespace detail

  // ===========================================================================
  // -- ThreadPool -------------------------------------------------------------
  // ===========================================================================

  /// A thread pool based on Boost.Asio's io context.
  class ThreadPool : private NonCopyable {
  public:

    class Strand;

    ThreadPool() : _work_to_do(_io_context) {}

    /// Stops the ThreadPool and joins all its threads.
    ~ThreadPool() {
      Stop();
    }

    /// Return the underlying boost::asio::io_context.
    auto &io_context() {
      return _io_context;
    }

    /// Post a task to the pool.
    template <typename FunctorT, typename ResultT = typename std::result_of<FunctorT()>::type>
    std::future<ResultT> Post(FunctorT &&functor) {
      return detail::PostPackagedTask<ResultT>(_io_context, std::forward<FunctorT>(functor));
    }

    /// Post a "raw" task to the pool. Similar to ThreadPool::Post but benefits
    /// performance at the expense of not packaging the task.
    ///
    /// @pre functor should not throw.
    template <typename FunctorT>
    void PostRaw(FunctorT &&functor) {
      detail::PostRawTask(_io_context, std::forward<FunctorT>(functor));
    }

    /// Launch threads to run tasks asynchronously. Launch specific number of
    /// threads if @a worker_threads is provided, otherwise use all available
    /// hardware concurrency.
    void AsyncRun(size_t worker_threads) {
      _workers.CreateThreads(worker_threads, [this]() { Run(); });
    }

    /// @copydoc AsyncRun(size_t)
    void AsyncRun() {
      AsyncRun(std::thread::hardware_concurrency());
    }

    /// Run tasks in this thread.
    ///
    /// @warning This function blocks until the ThreadPool has been stopped.
    void Run() {
      _io_context.run();
    }

    /// Run tasks in this thread for an specific @a duration.
    ///
    /// @warning This function blocks until the ThreadPool has been stopped, or
    /// until the specified time duration has elapsed.
    void RunFor(time_duration duration) {
      _io_context.run_for(duration.to_chrono());
    }

    /// Stop the ThreadPool and join all its threads.
    void Stop() {
      _io_context.stop();
      _workers.JoinAll();
    }

    Strand MakeStrand();

  private:

    boost::asio::io_context _io_context;

    boost::asio::io_context::work _work_to_do;

    ThreadGroup _workers;
  };

  // ===========================================================================
  // -- ThreadPool::Strand -----------------------------------------------------
  // ===========================================================================

  /// Provides the ability to post tasks to the pool with the guarantee that
  /// none of those tasks will execute concurrently.
  ///
  /// A wrap around Boost.Asio's io_context::strand.
  class ThreadPool::Strand : private MovableNonCopyable {
  public:

    /// Return the underlying boost::asio::io_context::strand.
    auto &strand() {
      return _strand;
    }

    /// @copydoc ThreadPool::Post
    template <typename FunctorT, typename ResultT = typename std::result_of<FunctorT()>::type>
    std::future<ResultT> Post(FunctorT &&functor) {
      return detail::PostPackagedTask<ResultT>(_strand, std::forward<FunctorT>(functor));
    }

    /// @copydoc ThreadPool::PostRaw
    template <typename FunctorT>
    void PostRaw(FunctorT &&functor) {
      detail::PostRawTask(_strand, std::forward<FunctorT>(functor));
    }

  private:

    friend class ThreadPool;

    Strand(boost::asio::io_context &io_context) : _strand(io_context) {}

    boost::asio::io_context::strand _strand;
  };

  inline ThreadPool::Strand ThreadPool::MakeStrand() {
    return _io_context;
  }

} // namespace carla
