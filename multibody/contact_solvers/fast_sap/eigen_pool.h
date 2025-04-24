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

  /* Adds element of the specified size and returns mutable to it. */
  ElementView Add(int rows, int cols) { return storage_.Add(rows, cols); }

  /* Adds new element and copies `data` into it.
   @returns mutable view to the new element. */
  ElementView AddAndCopy(const EigenType& data) {
    return storage_.AddAndCopy(data);
  }

  /* Sugar to add a data set into the pool. */
  // TODO(amcastro-tri): Consider more efficient, all-at-once, allocation.
  void PushBack(const std::vector<EigenType>& data) {
    for (const auto& d : data) {
      AddAndCopy(d);
    }
  }

  /* Clears data. Capacity is not changed, and thus memory is not freed. */
  void Clear() { storage_.Clear(); }

  /* Returns the number of elements in the pool. */
  int size() const { return storage_.size(); }

  /* Returns the maximum number of Eigen objects that can be stored without
  additional dynamics memory allocation. */
  int elements_capacity() const { return storage_.elements_capacity(); }

  /* Returns the capacity to store scalars. */
  int scalars_capacity() { return storage_.scalars_capacity(); }

  /* Const access to the i-th element. */
  const ConstElementView operator[](int i) const {
    DRAKE_ASSERT(0 <= i && i < size());
    return storage_.at(i);
  }

  /* Non-const access to the i-th element. */
  ElementView operator[](int i) {
    DRAKE_ASSERT(0 <= i && i < size());
    return storage_.at(i);
  }

  const std::vector<Scalar>& data() const { return storage_.data_; }

 private:
  // TODO(amcastro-tri): Specialize Storage to fixed-size Eigen elements, so
  // that it simply resolves to std::vector<EigenType>. E.g.
  // std::vector<Vector3>.
  struct Storage {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Storage);

    struct ElementData {
      int index{0};  // index into Storage::data_.
      int rows{0};   // Number of rows.
      int cols{0};   // Number of columns.
    };

    // Index into data_ for the next Eigen element.
    int next_data_index_{0};

    // Contiguous storage for all Eigen objects in the pool.
    std::vector<Scalar> data_;

    // Properly sized maps to each element in the pool.
    std::vector<ElementData> blocks_;

    Storage() = default;

    void Clear() {
      next_data_index_ = 0;
      data_.clear();
      blocks_.clear();
    }

    // Capcity to store Eigen elements.
    int elements_capacity() const { return blocks_.capacity(); }

    // Capacity to store scalar entries across all elements.
    int scalars_capacity() const { return data_.capacity(); }

    // Adds new element and returns mutable view to it.
    ElementView Add(int rows, int cols) {
      const int index = size();
      const int size = rows * cols;
      data_.resize(data_.size() + size);
      blocks_.emplace_back(next_data_index_, rows, cols);
      next_data_index_ += size;
      return at(index);
    }

    // Adds new element and copies `data` into it.
    // @returns index to the new element.
    ElementView AddAndCopy(const EigenType& data) {
      return Add(data.rows(), data.cols()) = data;
      //return at(size());      
    }

#if 0
    void PushBack(const std::vector<EigenType>& data) {
      // Increase capacity if needed.
      int new_data_capacity = data_.size();
      for (const auto& d : data) {
        new_data_capacity += d.size();
      }
      data_.reserve(new_data_capacity);
      const size_t new_maps_capacity = maps_.size() + data.size();

      // If capacity changes, we must re-point the maps.
      if (maps_.capacity() < new_maps_capacity) {
        maps_.reserve(new_maps_capacity);
        Scalar* ptr = data_.data();  // The very first scalar.
        for (auto& m : maps_) {
          // Use placement new to re-point each map.
          new (&m) Eigen::Map<EigenType>(ptr, m.rows(), m.cols());
          ptr += m.size();
        }
      }

      // Append data.
      Scalar* ptr = data_.data() + data_.size();
      for (const auto& e : data) {
        data_.insert(data_.end(), e.data(), e.data() + e.size());
        maps_.emplace_back(ptr, e.rows(), e.cols());
        ptr += e.size();
      }
    }

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
#endif

    int size() const { return blocks_.size(); }
    ConstElementView at(int i) const {
      return ConstElementView(&data_.at(blocks_[i].index), blocks_[i].rows,
                              blocks_[i].cols);
    }
    ElementView at(int i) {
      return ElementView(&data_.at(blocks_[i].index), blocks_[i].rows,
                         blocks_[i].cols);
    }
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
