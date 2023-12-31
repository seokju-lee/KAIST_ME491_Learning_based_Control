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
    opponent = reinterpret_cast<raisim::ArticulatedSystem *>(world->getObject(opponentName_));
    ground = reinterpret_cast<raisim::Ground *>(world->getObject("ground"));
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
    obDim_ = 52;
    actionDim_ = nJoints_;
    actionMean_.setZero(actionDim_);
    actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);
    impulse_from_player = 0;
    impulse_from_oppo = 0;

    // action_set.push_back(Eigen::VectorXd::Zero(gcDim_));
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

    opponent->getState(opponentGc, opponentGv);
    oppoquat[0] = opponentGc[3];
    oppoquat[1] = opponentGc[4];
    oppoquat[2] = opponentGc[5];
    oppoquat[3] = opponentGc[6];
    raisim::quatToRotMat(oppoquat, opporot);
    oppobodyLinearVel_ = opporot.e().transpose()*opponentGv.segment(0, 3);
    oppobodyAngularVel_ = opporot.e().transpose()*opponentGv.segment(3, 3);

    relpos_ = rot.e().transpose()*(opponentGc.head(3) - gc_.head(3));    
    opporelpos_ = opporot.e().transpose()*(gc_.head(3) - opponentGc.head(3));

    obDouble_ << gc_.head(3), /// body pose
        rot.e().row(2).transpose(), /// body orientation
        bodyLinearVel_, bodyAngularVel_, /// body linear&angular velocity
        relpos_.head(2),
        gv_.tail(12),
        opponentGc.head(3), /// opponent pose
        opporot.e().row(2).transpose(), /// opponent body orientation
        oppobodyLinearVel_, oppobodyAngularVel_, /// oppponent body linear&angular velocity
        opporelpos_.head(2),
        opponentGv.tail(12);
  }

  inline bool player_die_terminate(raisim::World *world){
    if ((gc_[2] < 0.3) | (gc_.head(2).norm() > 3)) {
      return true;
    }
    return false;
  }

  inline bool opponent_die_terminate(raisim::World *world){
    for(auto& contact: opponent->getContacts()){
      if(contact.getPairObjectIndex() == ground->getIndexInWorld() &&
        contact.getlocalBodyIndex() == opponent->getBodyIdx("base")){
        return true;
      }
    }
    if (opponentGc.head(2).norm() > 3) {
      return true;
    }
    return false;
  }

  inline void recordReward(Reward *rewards) {
    Eigen::VectorXd target_vector, target_direction, base_direction;
    double cosine, roll, pitch, oppo_roll, oppo_pitch;
    target_direction = relpos_.head(2)/relpos_.head(2).norm();
    cosine = target_direction.dot(bodyLinearVel_.head(2));
   for(auto& oppcontact: opponent->getContacts()){
     if(oppcontact.getPairObjectIndex() == anymal_->getIndexInWorld() &&
       oppcontact.getlocalBodyIndex() == opponent->getBodyIdx("base")){
       rewards->record("impulse", (oppcontact.getContactFrame().e().transpose() * oppcontact.getImpulse().e()).norm());
     }
   }
    rewards->record("torque", anymal_->getGeneralizedForce().squaredNorm());
    rewards->record("heading", std::max(cosine, 0.0));
    rewards->record("joint_vel", gv_.tail(12).squaredNorm());
    rewards->record("opp_center", -exp(-opponentGc.head(2).norm()));
    rewards->record("center_dist", exp(-gc_.head(2).norm()));
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
    for(auto& contact: anymal_->getContacts()){
      if(contact.getPairObjectIndex() == ground->getIndexInWorld() &&
        contact.getlocalBodyIndex() == anymal_->getBodyIdx("base")){
        return true;
      }
    }
    if (gc_.head(2).norm() > 3) {
      return true;
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
  raisim::ArticulatedSystem *anymal_, *opponent;
  raisim::Ground *ground;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, opponentGc, opponentGv, pTarget_, pTarget12_, vTarget_, o_pTarget_;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_, relpos_;
  Eigen::Vector3d oppobodyLinearVel_, oppobodyAngularVel_, opporelpos_;
  raisim::Vec<4> quat, oppoquat;
  raisim::Mat<3, 3> rot, opporot;
  std::set<size_t> footIndices_;
  std::vector<Eigen::VectorXd> action_set;
  int obDim_ = 0, actionDim_ = 0;
  double impulse_from_player, impulse_from_oppo;
  double forwardVelRewardCoeff_ = 0.;
  double torqueRewardCoeff_ = 0.;
};

}
