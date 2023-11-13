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
    o_pTarget_.setZero(gcDim_);
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
    obDim_ = 49;
    actionDim_ = nJoints_;
    actionMean_.setZero(actionDim_);
    actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);
    // action_set.push_back(Eigen::VectorXd::Zero(gcDim_));
    // auto* ground = world->addGround();
    // ground->setName("ground");
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

  Eigen::VectorXd generateTraj(raisim::World *world){
    // action_set.push_back(pTarget_);
    // if (action_set.size() > 1){
    //   action_set.erase(action_set.begin());
    // }
    // return action_set;
    return pTarget_;
  }

  inline bool prev_advance(raisim::World *world, Eigen::VectorXd traj) {
    /// action scaling
    // if (traj.size() > 1){
    //   o_pTarget_.tail(nJoints_) = traj[1];
    // }
    o_pTarget_.tail(nJoints_) << 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
    anymal_->setPdTarget(o_pTarget_, vTarget_);
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

    // for (auto &contact: opponent->getContacts()){
    //   if((contact.getPairObjectIndex() == 2 &&
    //       contact.getlocalBodyIndex() == opponent->getBodyIdx("base")) |
    //       (opponentGc.head(2).norm() > 3)){
    //     rewaboutopo = 5.;
    //   }
    //   else{
    //     rewaboutopo = 0.;
    //   }
    //   if(contact.getPairObjectIndex() != 2 &&
    //       contact.getlocalBodyIndex() == opponent->getBodyIdx("base")){
    //     oppositetouch_ = 1;
    //   }
    //   else{
    //     oppositetouch_ = 0;
    //   }
    // }

    obDouble_ << gc_.head(3), /// body pose
        rot.e().row(2).transpose(), /// body orientation
        gc_.tail(12), /// joint angles
        bodyLinearVel_, bodyAngularVel_, /// body linear&angular velocity
        gv_.tail(12), /// joint velocity
        opponentGc.head(3), /// opponent pose
        opporot.e().row(2).transpose(), /// opponent body orientation
        oppobodyLinearVel_, oppobodyAngularVel_, /// oppponent body linear&angular velocity
        oppositetouch_;
  }

  inline void recordReward(Reward *rewards) {
    rewards->record("torque", anymal_->getGeneralizedForce().squaredNorm());
    rewards->record("forwardVel", std::min(1.0, bodyLinearVel_[0]));
    Eigen::VectorXd target_vector, target_direction, base_direction;
    double cosine;
    target_vector = opponentGc.head(2) - gc_.head(2);
    target_direction = target_vector/target_vector.norm();
    base_direction = gc_.head(2)/gc_.head(2).norm();
    cosine = target_direction.dot(base_direction);
    if(cosine < 0){
      // rewards->record("angularVel", std::min(0.5, bodyAngularVel_[2]));
      rewards->record("opponentOri", 0);
    }
    else{
      rewards->record("opponentOri", cosine);
    }
    for (auto &contact: anymal_->getContacts()) {
      for (auto &contact_: opponent->getContacts()){
        if(contact.getlocalBodyIndex() == anymal_->getBodyIdx("base") &&
          contact_.getlocalBodyIndex() == opponent->getBodyIdx("base")){
            rewards->record("opponentPos", 5.0);
        }
        else{
          rewards->record("opponentPos", exp(-(gc_.head(2)-opponentGc.head(2)).norm()));
        } 
      }
    }
    // if((gc_.head(2)-opponentGc.head(2)).norm() < 0.4){
    //   rewards->record("opponentPos", 5.0);
    // }
    
    // rewards->record("opponentPospenalty", exp(-(gc_.head(2)-opponentGc.head(2)).norm()));
    // rewards->record("opponentOut", rewaboutopo);
    // rewards->record("baseHeight", exp(-abs(gc_[2]-0.5)));
    // rewards->record("opponentOut", 1/(4*(1+exp(-opponentGc.head(2).squaredNorm()))));
    // rewards->record("opponentbaseHeight", 1/(1+exp(-opponentGc[2])));
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
    // std::cout << "Index: " << world->getObject("ground")->getIndexInWorld() << "\n";
    for (auto &contact: anymal_->getContacts()) {
      if (contact.getPairObjectIndex() == 2 &&
          contact.getlocalBodyIndex() != anymal_->getBodyIdx("LF_SHANK") &&
          contact.getlocalBodyIndex() != anymal_->getBodyIdx("RF_SHANK") &&
          contact.getlocalBodyIndex() != anymal_->getBodyIdx("LH_SHANK") &&
          contact.getlocalBodyIndex() != anymal_->getBodyIdx("RH_SHANK")) {
        for (auto &contact_: opponent->getContacts()){
          if(contact_.getlocalBodyIndex() == opponent->getBodyIdx("base")){
            return false;
          }
        }
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
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, opponentGc, opponentGv, pTarget_, pTarget12_, vTarget_, o_pTarget_;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  Eigen::Vector3d oppobodyLinearVel_, oppobodyAngularVel_;
  raisim::Vec<4> quat, oppoquat;
  raisim::Mat<3, 3> rot, opporot;
  std::set<size_t> footIndices_;
  std::vector<Eigen::VectorXd> action_set;
  int obDim_ = 0, actionDim_ = 0;
  double rewaboutopo, oppositetouch_;
  double forwardVelRewardCoeff_ = 0.;
  double torqueRewardCoeff_ = 0.;
};

}
