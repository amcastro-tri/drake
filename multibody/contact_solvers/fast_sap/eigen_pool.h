#pragma once

#include <vector>

// #include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace fast_sap {

/* A replacement for std::vector<MatrixX<T>> that offers a contiguous layout of
 memory.

 @tparam EigenType The type of the Eigen elements. E.g. MatrixXd,
 Vector3, etc.
 @pre EigenType must be an Eigen type derived from Eigen::MatrixBase. */
template <typename EigenType>
class EigenPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(EigenPool);

  static_assert(is_eigen_type<EigenType>::value, "Must be an Eigen type.");

  using Scalar = EigenType::Scalar;
  using ElementView = Eigen::Map<EigenType>;
  using ConstElementView = Eigen::Map<const EigenType>;

  /* Default constructor for an empty pool. */
  EigenPool() = default;

  /* Constructor for a pool of column vectors with the provided `sizes`.
   @pre EigenType::ColsAtCompileTime equals 1.
   @pre For fixed size vectors, "sizes" must match the compile-time size. */
  explicit EigenPool(const std::vector<int>& sizes) : storage_(sizes) {
    static_assert(EigenType::ColsAtCompileTime == 1,
                  "Only for column vectors.");
  }

  /* Constructor for a poll of Eigen::MatrixX with the specified number of rows
   and columns. */
  EigenPool(const std::vector<int>& rows, const std::vector<int>& cols)
      : storage_(rows, cols) {}

  /* Constructor for a pool of matrices with the provided `shapes`.
   @pre For fixed size matrices, "shapes" must match the compile-time sizes. */
  explicit EigenPool(const std::vector<std::pair<int, int>>& shapes) {
    DRAKE_ASSERT_VOID(ValidShapes(shapes));
    storage_ = Storage(shapes);
  }

  // Constructor for a pool of `size` fixed-size Eigen types.
  // @pre EigenType is a fixed size Eigen type.
  void Resize(int size) { storage_.Resize(size); }

  void Resize(const std::vector<int>& sizes) { storage_.Resize(sizes); }

  void Resize(const std::vector<int>& rows, const std::vector<int>& cols) {
    storage_.Resize(rows, cols);
  }

  /* Returns the number of elements in the pool. */
  int size() const { return storage_.size(); }

  /* Const access to the i-th element. */
  const ConstElementView& operator[](int i) const {
    DRAKE_ASSERT(0 <= i && i < size());
    return storage_.at(i);
  }

  /* Non-const access to the i-th element. */
  ElementView& operator[](int i) {
    DRAKE_ASSERT(0 <= i && i < size());
    return storage_.at(i);
  }

 private:
  // TODO(amcastro-tri): Specialize Storage to fixed-size Eigen elements, so
  // that it simply resolves to std::vector<EigenType>. E.g.
  // std::vector<Vector3>.
  struct Storage {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Storage);

    // TODO(amcastro-tri): For dynamic sizes, consider to allocate a char
    // buffer[] with the required amount of bytes (a std::vector<char>). The
    // allocate memory such that the sizeof(ElementView) bytes go first,
    // followed by the actual data bytes for that element. This would make the
    // entire view object, including its size and data, contiguous in memory.

    // Contiguous storage for all Eigen objects in the pool.
    std::vector<Scalar> data_;

    // Properly sized maps to each element in the pool.
    std::vector<ElementView> maps_;

    Storage() = default;

    void Resize(int num_elements) {
      static_assert(EigenType::SizeAtCompileTime != Eigen::Dynamic,
                    "Only for fixed-size Eigen types.");
      const int total = EigenType::SizeAtCompileTime * num_elements;
      data_.resize(total);
      maps_.reserve(num_elements);
      Scalar* ptr = data_.data();
      for (int i = 0; i < num_elements; ++i) {
        maps_.emplace_back(ptr);  // Fixed-size map.
        ptr += EigenType::SizeAtCompileTime;
      }
    }

    void Resize(const std::vector<int>& sizes) {
      static_assert(EigenType::ColsAtCompileTime == 1,
                    "Only for column vectors.");
      int total = 0;
      for (int s : sizes) total += s;
      data_.resize(total);

      maps_.reserve(ssize(sizes));
      Scalar* ptr = data_.data();
      for (int s : sizes) {
        maps_.emplace_back(ptr, s, 1);  // Always one column.
        ptr += s;
      }
    }

    void Resize(const std::vector<int>& rows, const std::vector<int>& cols) {
      DRAKE_ASSERT(rows.size() == cols.size());
      const int num_elements = ssize(rows);
      int total = 0;
      for (int i = 0; i < num_elements; ++i) {
        total += rows[i] * cols[i];
      }
      data_.resize(total);

      maps_.reserve(num_elements);
      Scalar* ptr = data_.data();
      for (int i = 0; i < num_elements; ++i) {
        maps_.emplace_back(ptr, rows[i], cols[i]);
        ptr += rows[i] * cols[i];
      }
    }

    explicit Storage(const std::vector<int>& sizes) {
      static_assert(EigenType::ColsAtCompileTime == 1,
                    "Only for column vectors.");
      Resize(sizes);
    }

    Storage(const std::vector<int>& rows, const std::vector<int>& cols) {
      Resize(rows, cols);
    }

    /* Constructor for a pool of matrices with the provided `shapes`.
     @pre For fixed size matrices, "shapes" must match the compile-time sizes.
   */
    explicit Storage(const std::vector<std::pair<int, int>>& shapes) {
      int total = 0;
      for (const auto& [rows, cols] : shapes) total += rows * cols;
      data_.resize(total);

      maps_.reserve(ssize(shapes));
      Scalar* ptr = data_.data();
      for (const auto& [rows, cols] : shapes) {
        maps_.emplace_back(ptr, rows, cols);
        ptr += rows * cols;
      }
    }

    int size() const { return maps_.size(); }
    const ConstElementView& at(int i) const { return maps_[i]; }
    ElementView& at(int i) { return maps_[i]; }
  };

  static void ValidShapes(const std::vector<std::pair<int, int>>& shapes) {
    for (const auto& [rows, cols] : shapes) {
      if ((EigenType::RowsAtCompileTime != Eigen::Dynamic &&
           rows != EigenType::RowsAtCompileTime) ||
          (EigenType::ColsAtCompileTime != Eigen::Dynamic &&
           cols != EigenType::ColsAtCompileTime)) {
        throw std::logic_error("Shape does not match compile time sizes.");
      }
    }
  }

  Storage storage_;
};

}  // namespace fast_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
