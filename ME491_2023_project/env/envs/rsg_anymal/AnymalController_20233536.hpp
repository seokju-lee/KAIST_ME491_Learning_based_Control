// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

#pragma once

#include <set>
#include "../../BasicEigenTypes.hpp"
#include "raisim/World.hpp"

namespace raisim {

/// change the class name and file name ex) AnymalController_20233536 -> AnymalController_STUDENT_ID
class AnymalController_20233536 {

 public:
  inline bool create(raisim::World *world) {
    anymal_ = reinterpret_cast<raisim::ArticulatedSystem *>(world->getObject(name_));

    /// get robot data
    gcDim_ = anymal_->getGeneralizedCoordinateDim();
    gvDim_ = anymal_->getDOF();
    nJoints_ = gvDim_ - 6;

    /// initialize containers
    gc_.setZero(gcDim_);
    gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_);
    gv_init_.setZero(gvDim_);
    opponentGc.setZero(gcDim_);
    opponentGv.setZero(gvDim_);
    pTarget_.setZero(gcDim_);
    vTarget_.setZero(gvDim_);
    pTarget12_.setZero(nJoints_);

    /// this is nominal configuration of anymal
    gc_init_ << 0, 0, 0.50, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;

    /// set pd gains
    Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
    jointPgain.setZero();
    jointPgain.tail(nJoints_).setConstant(50.0);
    jointDgain.setZero();
    jointDgain.tail(nJoints_).setConstant(0.2);
    anymal_->setPdGains(jointPgain, jointDgain);
    anymal_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 48;
    actionDim_ = nJoints_;
    actionMean_.setZero(actionDim_);
    actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);

    /// action scaling
    actionMean_ = gc_init_.tail(nJoints_);
    actionStd_.setConstant(0.1);

    /// indices of links that should not make contact with ground
    footIndices_.insert(anymal_->getBodyIdx("LF_SHANK"));
    footIndices_.insert(anymal_->getBodyIdx("RF_SHANK"));
    footIndices_.insert(anymal_->getBodyIdx("LH_SHANK"));
    footIndices_.insert(anymal_->getBodyIdx("RH_SHANK"));

    return true;
  }

  inline bool init(raisim::World *world) {
    return true;
  }

  inline bool advance(raisim::World *world, const Eigen::Ref<EigenVec> &action) {
    /// action scaling
    pTarget12_ = action.cast<double>();
    pTarget12_ = pTarget12_.cwiseProduct(actionStd_);
    pTarget12_ += actionMean_;
    pTarget_.tail(nJoints_) = pTarget12_;
    anymal_->setPdTarget(pTarget_, vTarget_);
    return true;
  }

  inline bool reset(raisim::World *world, double theta) {
    if (playerNum_ == 0) {
      gc_init_.head(3) << 1.5 * std::cos(theta), 1.5 * std::sin(theta), 0.5;
      gc_init_.segment(3, 4) << cos((theta - M_PI) / 2), 0, 0, sin((theta - M_PI) / 2);
    }
    else {
      gc_init_.head(3) << 1.5 * std::cos(theta + M_PI), 1.5 * std::sin(theta + M_PI), 0.5;
      gc_init_.segment(3, 4) << cos(theta / 2), 0, 0, sin(theta / 2);
    }
    anymal_->setState(gc_init_, gv_init_);
    return true;
  }

  inline void updateObservation(raisim::World *world) {
    anymal_->getState(gc_, gv_);
    quat[0] = gc_[3];
    quat[1] = gc_[4];
    quat[2] = gc_[5];
    quat[3] = gc_[6];
    raisim::quatToRotMat(quat, rot);
    bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
    bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);

    auto opponent = reinterpret_cast<raisim::ArticulatedSystem *>(world->getObject(opponentName_));
    opponent->getState(opponentGc, opponentGv);
    oppoquat[0] = opponentGc[3];
    oppoquat[1] = opponentGc[4];
    oppoquat[2] = opponentGc[5];
    oppoquat[3] = opponentGc[6];
    raisim::quatToRotMat(oppoquat, opporot);
    oppobodyLinearVel_ = opporot.e().transpose()*opponentGv.segment(0, 3);
    oppobodyAngularVel_ = opporot.e().transpose()*opponentGv.segment(3, 3);

    obDouble_ << gc_.head(3), /// body pose
        rot.e().row(2).transpose(), /// body orientation
        gc_.tail(12), /// joint angles
        bodyLinearVel_, bodyAngularVel_, /// body linear&angular velocity
        gv_.tail(12), /// joint velocity
        opponentGc.head(3), /// opponent pose
        opporot.e().row(2).transpose(), /// opponent body orientation
        oppobodyLinearVel_, oppobodyAngularVel_; /// oppponent body linear&angular velocity
  }

  inline void recordReward(Reward *rewards) {
    rewards->record("torque", anymal_->getGeneralizedForce().squaredNorm());
    if((rot.e().transpose()*opponentGc.head(3))[0] > 0){
      rewards->record("forwardVel", std::min(1.0, bodyLinearVel_[0]));
    }
    if((rot.e().transpose()*opponentGc.head(3))[0] < 0){
      rewards->record("backwardVel", std::max(-1.0, bodyLinearVel_[0]));
    }
    if((rot.e().transpose()*opponentGc.head(3))[1] > 0){
      rewards->record("rightVel", std::min(0.5, bodyLinearVel_[1]));
    }
    if((rot.e().transpose()*opponentGc.head(3))[1] < 0){
      rewards->record("leftVel", std::max(-0.5, bodyLinearVel_[1]));
    }
    Eigen::Vector2f err;
    err << gc_[0]-(opponentGc[0]-0.3), gc_[1]-opponentGc[1];
    rewards->record("opponentBase", exp(-err.squaredNorm()));
    rewards->record("centerdist", 1/(1+exp(gc_.head(2).squaredNorm())));
    rewards->record("opponentOut", 1/(4*(1+exp(-opponentGc.head(2).squaredNorm()))));
    rewards->record("opponentbaseHeight", 1/(4*(1+exp(-opponentGc[2]))));
  }

  inline const Eigen::VectorXd &getObservation() {
    return obDouble_;
  }

  void setName(const std::string &name) {
    name_ = name;
  }

  void setOpponentName(const std::string &name) {
    opponentName_ = name;
  }

  void setPlayerNum(const int &playerNum) {
    playerNum_ = playerNum;
  }

  inline bool isTerminalState(raisim::World *world) {
    for (auto &contact: anymal_->getContacts()) {
      if (footIndices_.find(contact.getlocalBodyIndex()) == footIndices_.end()) {
        return true;
      }
    }
    return false;
  }

  inline int getObDim() {
    return obDim_;
  }

  inline int getActionDim() {
    return actionDim_;
  }


 private:
  std::string name_, opponentName_;
  int gcDim_, gvDim_, nJoints_, playerNum_ = 0;
  raisim::ArticulatedSystem *anymal_;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, opponentGc, opponentGv, pTarget_, pTarget12_, vTarget_;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  Eigen::Vector3d oppobodyLinearVel_, oppobodyAngularVel_;
  raisim::Vec<4> quat, oppoquat;
  raisim::Mat<3, 3> rot, opporot;
  std::set<size_t> footIndices_;
  int obDim_ = 0, actionDim_ = 0;
  double forwardVelRewardCoeff_ = 0.;
  double torqueRewardCoeff_ = 0.;
};

}
