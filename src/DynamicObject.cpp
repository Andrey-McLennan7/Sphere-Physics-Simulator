#include "DynamicObject.h"
#include "Utility.h"
#include "CollisionPlane.h"
#include <exception>

DynamicObject::DynamicObject()
{
	this->_force = glm::vec3(0.0f, 0.0f, 0.0f);
	this->_velocity= glm::vec3(0.0f, 0.0f, 0.0f);

	this->setMass(1.0f);
	this->_bRadius = 1.0f;
	this->_start = true;
	this->_position = glm::vec3(0.0f, 0.0f, 0.0f);

	this->_previous_position = glm::vec3(0.0f, 0.0f, 0.0f);
	this->_acceleration = glm::vec3(0.0f, 0.0f, 0.0f);
	this->_elasticity = 0.0f;

	this->_torque = glm::vec3(0.0f, 0.0f, 0.0f);
	this->_angular_velocity = glm::vec3(0.0f, 0.0f, 0.0f);
	this->_angular_momentum = glm::vec3(0.0f, 0.0f, 0.0f);

	this->_R = glm::mat3
	{
		1.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 1.0f
	};

	this->global_damping = 8.0f;
}

void DynamicObject::update(float deltaTs)
{
	// Step 1: clear all the forces and torques act on the object from the previous time step
	this->clearForce();
	this->clearTorque();

	// Step 2: compute each force that act on the object at the current time step
	glm::vec3 force{ this->_mass * this->_acceleration };
	this->addForce(force);

	// Step 3: compute collision and responses
	this->computeCollisionRes(deltaTs);

	// Step 4: integration
	//this->euler(deltaTs);  // Euler
	this->rk2(deltaTs);    // RK2
	//this->rk4(deltaTs);    // RK4
	//this->verlet(deltaTs); // Verlet

	this->setPosition(this->_position);

	glm::vec3 euler_angles{ this->getEulerAngles(this->_R) };
	float degree{ 180.0f / 3.141596f };

	this->setRotation((euler_angles.x * degree), (euler_angles.y * degree), (euler_angles.z * degree));
}

void DynamicObject::computeCollisionRes(float deltaTs)
{
	if (PHY_SPHERE == this->_type)
	{
		for (int i{ 0 }; i < this->otherPlanes.size(); ++i)
		{
			this->planeCollisionResponce(this->otherPlanes.at(i), deltaTs);
		}

		for (int i{ 0 }; i < this->otherObjects.size(); ++i)
		{
			this->sphereCollisionResponce(this->otherObjects.at(i));
		}
	}
}

void DynamicObject::euler(float deltaTs)
{
	this->_velocity += (this->_force / this->_mass) * deltaTs;

	this->_position += this->_velocity * deltaTs;

	this->AngularMotion(deltaTs);
}

void DynamicObject::rk2(float deltaTs)
{
	glm::vec3 force;
	glm::vec3 acceleration;
	glm::vec3 k0;
	glm::vec3 k1;

	// Evaluate once at t0
	force = this->_force;
	acceleration = force / this->_mass;
	k0 = deltaTs * acceleration;

	// Evaluate once at t0 + deltaTs/2.0 using half of k0
	force = this->_force + (k0 / 2.0f);
	acceleration = force / this->_mass;
	k1 = deltaTs * acceleration;

	// Evaluate once at t0 + deltaTs using k1
	this->_velocity += k1;
	this->_position += this->_velocity * deltaTs;

	this->AngularMotion(deltaTs);
}

void DynamicObject::rk4(float deltaTs)
{
	glm::vec3 force;
	glm::vec3 acceleration;
	glm::vec3 k0;
	glm::vec3 k1;
	glm::vec3 k2;
	glm::vec3 k3;

	// Evaluate once at t0 to find k0
	force = this->_force;
	acceleration = force / this->_mass;
	k0 = deltaTs * acceleration;

	// Evaluate twice at t0 + deltaTs/2.0 using half of k0 and half of k1
	force = this->_force + (k0 / 2.0f);
	acceleration = force / this->_mass;
	k1 = deltaTs * acceleration;

	force = this->_force + (k1 / 2.0f);
	acceleration = force / this->_mass;
	k2 = deltaTs * acceleration;

	// Evaluate once at t0 + deltaTs using k2
	force = this->_force + k2;
	acceleration = force / this->_mass;
	k3 = deltaTs * acceleration;

	// Evaluate at t0 + deltaTs using the weighted sum of k0, k1, k2, and k3
	this->_velocity += ((k0 + 2.0f) * (k1 + 2.0f) * k2 + k3) / 6.0f;
	// Update position
	this->_position += this->_velocity * deltaTs;

	this->AngularMotion(deltaTs);
}

void DynamicObject::verlet(float deltaTs)
{
	glm::vec3 acceleration{ this->_force / this->_mass };

	this->_previous_position = this->_position - this->_velocity * deltaTs
		+ 0.5f * acceleration * deltaTs * deltaTs;

	this->_position = 2.0f * this->_position - this->_previous_position
		+ acceleration * deltaTs * deltaTs;

	this->_velocity = (this->_position - this->_previous_position) / (2.0f * deltaTs);

	this->_velocity += acceleration * deltaTs;

	this->AngularMotion(deltaTs);
}

void DynamicObject::sphereCollisionResponce(DynamicObject* other)
{
	if (!other)
	{
		return;
	}

	float elasticity{ 0.66f };

	glm::vec3 position_distance{ this->_position - other->getPosition() };
	glm::vec3 n{ glm::normalize(position_distance) };

	glm::vec3 relative_velocity{ this->_velocity - other->getVelocity() };

	glm::vec3 cp;

	float distance{ glm::length(position_distance) };

	if (PFG::SphereToSphereCollision(this->_position, other->getPosition(), this->_bRadius, other->getBoundingRadius(), cp))
	{
		this->applyImpulseResponces(this, other);

		float penetration{ abs((this->_bRadius + other->getBoundingRadius()) - distance) };

		float inverse_mass1 = 1.0f / this->_mass;
		float inverse_mass2 = 1.0f / other->getMass();

		float total_inverse_mass{ inverse_mass1 + inverse_mass2 };
		float total_mass{ this->_mass * other->getMass() };

		glm::vec3 mov{ this->_position + penetration * n * inverse_mass1 / total_inverse_mass };
		this->setPosition(mov);

		mov = other->getPosition() + penetration * n * inverse_mass2 / total_inverse_mass;
		other->setPosition(mov);
	}
}

void DynamicObject::planeCollisionResponce(CollisionPlane* other, float deltaTs)
{
	float e{ 0.5f };

	glm::vec3 c1{ this->_position + this->_velocity * deltaTs };
	glm::vec3 ci{ glm::vec3(0.0f) };

	float inverse_mass1{ 1.0f / this->_mass };
	float inverse_mass2{ 0.0f };

	glm::vec3 velA{ this->_velocity + glm::cross(this->_angular_velocity, this->_bRadius * other->getNormal()) };
	glm::vec3 velB{ other->getVelocity() };
	glm::vec3 relative_velocity{ velA - velB };

	glm::vec3 cross1{ glm::cross(this->_bRadius * other->getNormal(), other->getNormal()) };
	cross1 = glm::cross(this->_inertia_tensor_inverse * cross1, this->_bRadius * other->getNormal());
	float total_inverse_mass{ inverse_mass1 + inverse_mass2 + glm::dot(cross1, other->getNormal()) };

	if (PFG::MovingSphereToPlaneCollision(other->getNormal(), this->_position, c1, other->getPoint(), this->_bRadius, ci))
	{
		// Do impulse based response here
		glm::vec3 plane_velocity{ glm::vec3(0.0f, 0.0f, 0.0f) };
		float collision_impulse{ -(1 + e) * glm::dot(relative_velocity, other->getNormal()) / total_inverse_mass };
		glm::vec3 collision_impulse_vector{ collision_impulse * other->getNormal() };

		this->_velocity += collision_impulse_vector / this->_mass;
		this->_angular_velocity += this->_inertia_tensor_inverse * glm::cross(this->_bRadius * other->getNormal(), collision_impulse_vector);

		// Add constant normal force
		glm::vec3 gravity_force{ glm::vec3(0.0f, this->_mass * 9.8f, 0.0f) };
		glm::vec3 normal_force{ glm::dot(gravity_force, other->getNormal()) * other->getNormal() };
		this->addForce(normal_force);

		// Compute friction force
		float d_mu{ 0.66f };
		glm::vec3 forward_relative_velocity{ relative_velocity - glm::dot(relative_velocity, other->getNormal()) * other->getNormal() };

		glm::vec3 friction_force{ this->frictionForce(relative_velocity, other->getNormal(), normal_force, d_mu) };
		glm::vec3 torque_arm{ this->_bRadius * other->getNormal() };
		glm::vec3 torque{ glm::cross(friction_force, torque_arm) };

		// Add global damping
		torque -= this->_angular_momentum * this->global_damping;
		this->addForce(friction_force);

		// if moving forward, add torque
		if (glm::length(forward_relative_velocity) - glm::length(friction_force / this->_mass) * deltaTs > 0.0f)
		{
			this->addForce(-friction_force);
			this->addTorque(torque);
		}
	}
}

glm::vec3 DynamicObject::frictionForce(glm::vec3 relative_velocity, glm::vec3 contact_normal, glm::vec3 force_normal, float mu)
{
	glm::vec3 friction_force;
	glm::vec3 forward_relative_velocity
	{
		relative_velocity - glm::dot(relative_velocity, contact_normal) * contact_normal
	};

	if (glm::length(forward_relative_velocity) > 1e-6f)
	{
		// Get a normalised vector as the direction of travel
		glm::vec3 forward_direction{ glm::normalize(forward_relative_velocity) };

		// Frection direction is in the opposite direction of travel
		glm::vec3 friction_direction = -forward_direction;
		friction_force = friction_direction * mu * glm::length(force_normal);

		return friction_force;
	}
	else return glm::vec3(0.0f);
}

glm::vec3 DynamicObject::getEulerAngles(glm::mat3 R)
{
	float value{ (R[0][0] * R[0][0]) + (R[1][0] * R[1][0]) };
	float sy{ sqrt(value) };

	bool singular{ sy < 1e-6 };

	float x{ 0.0f }, y{ 0.0f }, z{ 0.0f };
	if (!singular)
	{
		x = atan2(R[2][1], R[2][2]);
		y = atan2(-R[2][0], sy);
		z = atan2(R[1][0], R[0][0]);
	}
	else
	{
		x = atan2(-R[1][2], R[1][1]);
		y = atan2(-R[2][0], sy);
		z = 0;
	}

	return glm::vec3(x, y, z);
}

void DynamicObject::AngularMotion(float deltaTs)
{
	// Update Angular Motion
	this->_angular_momentum += this->_torque * deltaTs;
	this->computeInverseInertiaTensor();

	// Update Angular Velocity
	this->_angular_velocity = this->_inertia_tensor_inverse * this->_angular_momentum;

	// Construct skew matrix omega star
	glm::mat3 omega_star
	{
		glm::mat3
		{
			0.0f, -this->_angular_velocity.z, this->_angular_velocity.y,
			this->_angular_velocity.z, 0.0f, -this->_angular_velocity.x,
			-this->_angular_velocity.y, this->_angular_velocity.x, 0.0f
		}
	};

	// Update Rotation Matrix
	this->_R += omega_star * this->_R * deltaTs;
}

void DynamicObject::applyImpulseResponces(DynamicObject* objA, DynamicObject* objB)
{
	glm::vec3 position_distance{ objA->getPosition() - objB->getPosition()};
	glm::vec3 n{ glm::normalize(position_distance) };

	glm::vec3 relative_velocity{ objA->getVelocity() - objB->getVelocity() };

	float elasticity{ objA->getElasticity() * objB->getElasticity() };

	float one_over_massA{ 1.0f / objA->getMass() };
	float one_over_massB{ 1.0f / objB->getMass() };

	float J_numerator{ -(1.0f + elasticity) * glm::dot(relative_velocity, n) };

	float total_inverse_mass{ one_over_massA + one_over_massB };
	float J{ J_numerator / total_inverse_mass };

	glm::vec3 collision_impulse_vector{ J * n };

	// Object A
	glm::vec3 velocity{ objA->getVelocity() + collision_impulse_vector * one_over_massA };
	objA->setVelocity(velocity);

	// Object B
	velocity = objB->getVelocity() - collision_impulse_vector * one_over_massB;
	objB->setVelocity(velocity);
}

void DynamicObject::computeInverseInertiaTensor()
{
	this->_inertia_tensor_inverse = this->_R * this->_body_inertia_tensor_inverse * glm::transpose(this->_R);
}

void DynamicObject::setPosition(glm::vec3 pos)
{
	this->setPosition(pos.x, pos.y, pos.z);
}

void DynamicObject::setPosition(float x, float y, float z)
{
	this->_position = glm::vec3(x, y, z);

	Object::setPosition(x, y, z);
}

void DynamicObject::setMass(float mass)
{
	int value = (int)mass;

	if (value == 0)
		throw std::exception();

	this->_mass = mass;

	this->setInertiaTensor();
}

void DynamicObject::setInertiaTensor()
{
	float matI{ (float)((2.0f / 5.0f) * this->_mass * std::pow(this->_bRadius, 2)) };

	// Compute the sphere's body intertia
	glm::mat3 body_inertia = glm::mat3
	{
		matI, 0.0f, 0.0f,
		0.0f, matI, 0.0f,
		0.0f, 0.0f, matI
	};

	// Inverse body inertia
	this->_body_inertia_tensor_inverse = glm::inverse(body_inertia);

	// Compute inertia tensor inverse
	this->computeInverseInertiaTensor();
}

void DynamicObject::setType(int type)
{
	this->_type = type;
	Object::setType(this->_type);
}

void DynamicObject::setScale(glm::vec3 scl)
{
	this->setScale(scl.x, scl.y, scl.z);
}

void DynamicObject::setScale(float x, float y, float z)
{
	this->_scale = glm::vec3(x, y, z);
	Object::setScale(x, y, z);
}