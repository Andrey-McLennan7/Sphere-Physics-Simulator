#pragma once

#ifndef _DYNAMIC_OBJECT_H_
#define _DYNAMIC_OBJECT_H_

#include "phy.h"
#include "glm/glm.hpp"
#include <vector>

class DynamicObject : public Object
{
private:
	glm::vec3 _acceleration;
	glm::vec3 _force;
	glm::vec3 _velocity;
	glm::vec3 _position;
	glm::vec3 _scale;
	glm::vec3 _previous_position;

	glm::vec3 _torque;
	glm::vec3 _angular_velocity;
	glm::vec3 _angular_momentum;
	glm::mat3 _inertia_tensor_inverse;
	glm::mat3 _body_inertia_tensor_inverse;
	glm::mat3 _R;

	float global_damping;
	float _mass;
	float _bRadius;
	bool _start;

	float _elasticity;

	std::vector<DynamicObject*> otherObjects;
	std::vector<class CollisionPlane*> otherPlanes;

	int _type;

	// Update Object overall
	void addForce(const glm::vec3 force) { this->_force += force; }
	void clearForce() { this->_force = glm::vec3(0.0f); }
	void addTorque(const glm::vec3 torque) { this->_torque += torque; }
	void clearTorque() { this->_torque = glm::vec3(0.0f); }

	// Calculate the collision
	void computeCollisionRes(float deltaTs);
	void sphereCollisionResponce(DynamicObject* other);
	void planeCollisionResponce(class CollisionPlane* _other, float deltaTs);

	// Calculate Angular Velocity
	void setInertiaTensor();
	glm::vec3 frictionForce(glm::vec3 relative_velocity, glm::vec3 contact_normal, glm::vec3 force_normal, float mu);
	glm::vec3 getEulerAngles(glm::mat3 R);

	void AngularMotion(float deltaTs);

	void applyImpulseResponces(DynamicObject* objA, DynamicObject* objB);
	void computeInverseInertiaTensor();

	// Calculate Linear Velocity
	void euler(float deltaTs);
	void rk2(float deltaTs);
	void rk4(float deltaTs);
	void verlet(float deltaTs);

public:

	DynamicObject();

	void update(float deltaTs);

	// Setters
	void setPosition(glm::vec3 pos);
	void setPosition(float x, float y, float z);
	void setType(int type);
	void setScale(glm::vec3 scl);
	void setScale(float x, float y, float z);

	void setVelocity(glm::vec3 velocity)  { this->_velocity = velocity;  }
	void setAcceleration(glm::vec3 accel) { this->_acceleration = accel; }
	void setForce(glm::vec3 force)		  { this->_force = force;		 }
	void setBoundingRadius(float bRadius) { this->_bRadius = bRadius;	 }
	void setElasticity(float elas)		  { this->_elasticity = elas;	 }

	void setCollisionObject(DynamicObject* other)		{ this->otherObjects.push_back(other); }
	void setCollisionPlane(class CollisionPlane* other) { this->otherPlanes.push_back(other);  }
	void setMass(float mass);

	// Getters
	const glm::vec3 getPosition()	  const	  { return this->_position;	    }
	const glm::vec3 getVelocity()	  const	  { return this->_velocity;	    }
	const glm::vec3 getAcceleration() const   { return this->_acceleration; }
	const glm::vec3 getForce()		  const	  { return this->_force;		}

	const float getBoundingRadius()	  const   { return this->_bRadius;	    }
	const float getMass()			  const	  { return this->_mass;		    }
	const float getElasticity()		  const   { return this->_elasticity;   }

	const int getType() const { return this->_type; }
};

#endif