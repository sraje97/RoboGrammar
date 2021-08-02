#pragma once

#include <Eigen/Dense>
#include <memory>
#include <robot_design/sim.h>
#include <robot_design/types.h>
#include <vector>

namespace robot_design {

// Provides a way to write non-template functions taking Eigen objects as
// as parameters while limiting the number of copies.
using Eigen::Ref;

// ValueEstimator classes take in simulation object, observations & final
// observations to either get the observation object or estimate values
// of final states
class ValueEstimator {
public:
  virtual ~ValueEstimator() {}
  // 'const' means this function is a 'const member function' so it will
  // not modify the object
  // '=0' means its a pure virtual function meaning subclasses have to
  // implement this function
  virtual int getObservationSize() const = 0;
  virtual void getObservation(
      const Simulation &sim, Ref<VectorX> obs) const = 0;
  virtual void estimateValue(
      const MatrixX &obs, Ref<VectorX> value_est) const = 0;
  virtual void train(const MatrixX &obs, const Ref<const VectorX> &value) = 0;
};

// Inherit ValueEstimator class except override the getObservationSize func
class NullValueEstimator : public ValueEstimator {
public:
  NullValueEstimator() {}
  virtual ~NullValueEstimator() {}
  // const override says it inherits a virtual function of same name & signature
  // and won't change any member variables or call other functions that might
  virtual int getObservationSize() const override { return 0; }
  virtual void getObservation(
      const Simulation &sim, Ref<VectorX> obs) const override {}
  virtual void estimateValue(
      const MatrixX &obs, Ref<VectorX> value_est) const override {
    value_est = VectorX::Zero(obs.cols());
  }
  virtual void train(
      const MatrixX &obs, const Ref<const VectorX> &value) override {}
};

} // namespace robot_design
