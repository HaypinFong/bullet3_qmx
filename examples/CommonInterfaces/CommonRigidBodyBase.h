
#ifndef COMMON_RIGID_BODY_BASE_H
#define COMMON_RIGID_BODY_BASE_H

#include "btBulletDynamicsCommon.h"
#include "CommonExampleInterface.h"
#include "CommonGUIHelperInterface.h"
#include "CommonRenderInterface.h"
#include "CommonCameraInterface.h"
#include "CommonGraphicsAppInterface.h"
#include "CommonWindowInterface.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"

#define FORBID_DRAG_CONSTRAINT	false	// 260616FHP

struct CommonRigidBodyBase : public CommonExampleInterface
{
	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
	btBroadphaseInterface* m_broadphase;
	btCollisionDispatcher* m_dispatcher;
	btConstraintSolver* m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btDiscreteDynamicsWorld* m_dynamicsWorld;

	//data for picking objects
	class btRigidBody* m_pickedBody;
	//260616FHP
	class btRigidBody* m_pickedBodyOnce;	// unlike pickedBody clear after release mousebutton, keep until next pickedBody
	// keyboard pushforce param
	bool m_keyUp = false;
	bool m_keyDown = false;
	bool m_keyLeft = false;
	bool m_keyRight = false;
	const btScalar KEY_FORCE = 10.0f;  // axial push force at keyboard down once
	const btScalar KEY_DAMP = 8;    // velocity damp to avoid goahead forever
	const btScalar TORQUE_STRENGTH = 20.0f;
	const btScalar ANGULAR_DAMP_COEFF = 30.f;
	const btScalar bounce = 0.15f;
	const btScalar frictionMu = 0.35f;
	class btTypedConstraint* m_pickedConstraint;
	int m_savedState;
	btVector3 m_oldPickingPos;
	btVector3 m_hitPos;
	btScalar m_oldPickingDist;
	struct GUIHelperInterface* m_guiHelper;

	CommonRigidBodyBase(struct GUIHelperInterface* helper)
		: m_broadphase(0),
		  m_dispatcher(0),
		  m_solver(0),
		  m_collisionConfiguration(0),
		  m_dynamicsWorld(0),
		  m_pickedBody(0),
		  m_pickedBodyOnce(0),
		  m_pickedConstraint(0),
		  m_guiHelper(helper)
	{
	}
	virtual ~CommonRigidBodyBase()
	{
	}

	btDiscreteDynamicsWorld* getDynamicsWorld()
	{
		return m_dynamicsWorld;
	}

	virtual void createEmptyDynamicsWorld()
	{
		///collision configuration contains default setup for memory, collision setup
		m_collisionConfiguration = new btDefaultCollisionConfiguration();
		//m_collisionConfiguration->setConvexConvexMultipointIterations();

		///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
		m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

		m_broadphase = new btDbvtBroadphase();

		///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
		btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
		m_solver = sol;

		m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);

		m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
	}

	void FixCubeGroundPenetration(btRigidBody* body)
	{
		// 唤醒物体，休眠时才能受力
		body->setActivationState(ACTIVE_TAG);

		btTransform trans = body->getWorldTransform();
		btVector3 center = trans.getOrigin();
		btVector3 min_bb, max_bb;
		// 立方体碰撞盒半高
		body->getCollisionShape()->getAabb(trans, min_bb, max_bb);  // 世界坐标系下包围盒，地面是世界坐标系Y=0
		btScalar halfY = 0.5 * (max_bb - min_bb).y();
		// 立方体底部世界Y坐标
		btScalar cubeBottomY = center.y() - halfY;

		// 采样立方体中心正下方地形高度
		btScalar groundY = 0;  //GetTerrainHeight(center.x(), center.z());
		// 穿透阈值，防止微小抖动
		const btScalar epsilon = btScalar(0.01);

		btVector3 linVel = body->getLinearVelocity();	// 世界坐标系

		// 分支1：立方体底部低于地面 → 穿透，强制抬升并抵消向下速度
		if (cubeBottomY < groundY - epsilon)
		{
			// 计算需要抬高的偏移量
			btScalar deltaUp = groundY - cubeBottomY + epsilon;
			// 修正物体中心位置，刚好落在地面上方
			center.setY(center.y() + deltaUp);
			trans.setOrigin(center);
			body->setWorldTransform(trans);

			// 撞击地面时，竖直速度反向乘以弹性系数
			if (linVel.y() < 0)
			{
				//linVel.setY(0);
				linVel.setY(-linVel.y() * bounce);
				body->setLinearVelocity(linVel);
			}
		}
		// 分支2：立方体贴地（底部贴近地面，无穿透），仅施加摩擦，不修改位置
		else if (cubeBottomY <= groundY + epsilon)
		{
			/**
			* 仿地表滑动摩擦逻辑
			* Ffric = -μ·m·g·vxz/|vxz|
			* μ: 地表摩擦系数，泥土0.3，水泥0.8，冰0.1
			* m: 刚体质量
			* g: 重力加速度
			* vxz: 水平xz平面速度(消除竖直下落速度干扰)
			* 负号: 力与滑动速度反向，持续减速实现摩擦阻尼
			*/
			// 分离水平速度XZ，剔除竖直Y速度（下落不产生摩擦）
			btVector3 vel = linVel;
			btVector3 velHorizontal(vel.x(), 0, vel.z());
			btScalar horSpeed = velHorizontal.length();

			// 速度极小直接清零，防止微小抖动
			if (horSpeed < 0.001f)
			{
				body->setLinearVelocity(btVector3(0, vel.y(), 0));
				return;
			}

			// 计算滑动摩擦力
			btScalar mass = body->getInvMass() > 0 ? 1.0f / body->getInvMass() : 0;
			btScalar gravityMag = 9.8f;
			btScalar normalForce = mass * gravityMag;  // 法向压力 = mg
			btScalar frictionForceMag = frictionMu * normalForce;

			// 1 摩擦力方向：与水平速度反向单位向量 × 摩擦力大小
			btVector3 frictionForce = -(velHorizontal / horSpeed) * frictionForceMag;

			// 2 叠加地表滚动阻尼阻力
			//frictionForce -= velHorizontal * KEY_DAMP;

			// 在质心施加全局坐标系阻力（模拟滑动摩擦）
			body->applyCentralForce(frictionForce);

			// 3 施加角阻尼，削弱旋转
			btVector3 worldAngVel = body->getAngularVelocity();  // 世界坐标系
			worldAngVel.setX(0);                                 // 只施加Y轴旋转角阻尼
			worldAngVel.setZ(0);
			if (worldAngVel.length() > 1e-3)
			{
				btVector3 worldDampTorque = worldAngVel * (-ANGULAR_DAMP_COEFF);
				body->applyTorque(worldDampTorque);
			}
		}
		// 分支3：立方体在空中，无操作，自由落体
	}

	virtual void stepSimulation(float deltaTime)
	{
		if (m_dynamicsWorld)
		{
			// 0 前置修正：所有立方体落地防穿透、落地施加摩擦
			btCollisionObjectArray& objs = m_dynamicsWorld->getCollisionObjectArray();
			for (int i = 0; i < objs.size(); i++)
			{
				btRigidBody* body = btRigidBody::upcast(objs[i]);
				if (body)
					FixCubeGroundPenetration(body);
			}

			bool pressed_arrow_key = false;
			pressed_arrow_key |= m_keyUp;
			pressed_arrow_key |= m_keyDown;
			pressed_arrow_key |= m_keyLeft;
			pressed_arrow_key |= m_keyRight;
			if (pressed_arrow_key && m_pickedBodyOnce != nullptr && !m_pickedBodyOnce->isStaticObject())
			{
				btRigidBody* body = m_pickedBodyOnce;
				// 唤醒物体，休眠时才能受力
				body->setActivationState(ACTIVE_TAG);
				// 获取刚体旋转基矩阵：局部空间 → 世界空间转换
				btMatrix3x3 basis = body->getWorldTransform().getBasis();
				btVector3 vel = body->getLinearVelocity();  // 世界坐标系
				btVector3 vel_local = basis.inverse() * vel;
				btVector3 velHorizontal_local(vel_local.x(), 0, vel_local.z());
				btScalar horSpeed_local = velHorizontal_local.length();

				int countArrowKeyPressed = 0;
				countArrowKeyPressed += m_keyUp    ? 1 : 0;
				countArrowKeyPressed += m_keyDown  ? 1 : 0;
				countArrowKeyPressed += m_keyLeft  ? 1 : 0;
				countArrowKeyPressed += m_keyRight ? 1 : 0;
				// 施加局部坐标系下转向力矩(当且仅当同时有两个相邻方向键被按下时,同时按下上下键或左右键不产生扭矩)
				if (countArrowKeyPressed == 2 && !(m_keyUp && m_keyDown) && !(m_keyLeft && m_keyRight))
				{
					// 1 施加局部Y轴力矩
					int itorqueOriY = -1;
					if ((m_keyUp && m_keyLeft) || (m_keyDown && m_keyRight))
						itorqueOriY = 1;
					
					// 局部 Y 负方向 = 俯视顺时针
					btVector3 localTorque(0, TORQUE_STRENGTH * itorqueOriY, 0);
					btVector3 worldTorque = basis * localTorque;
					// 直接施加局部力矩
					body->applyTorque(worldTorque);
				}
				else if (countArrowKeyPressed == 1 && (m_keyLeft || m_keyRight) && horSpeed_local > 1e-1)
				{
					// 在有局部坐标系水平速度的前提下按下左或右键也允许产生扭矩
					
					// 1 施加局部Y轴力矩
					int itorqueOriY = m_keyLeft ?1:- 1;

					// 局部 Y 负方向 = 俯视顺时针
					btVector3 localTorque(0, TORQUE_STRENGTH * itorqueOriY, 0);
					btVector3 worldTorque = basis * localTorque;
					// 直接施加局部力矩
					body->applyTorque(worldTorque);
				}

				// 施加局部坐标系下轴心推力(必须有前进或后退键按下,允许同时按下前进后退抵消,仅单纯按下左或右或同时按下左右不产生推力)
				if (m_keyUp || m_keyDown)
				{
					// 2 施加局部坐标系下轴向力
					btVector3 localForce(0, 0, 0);

					// 上：-Z 轴；下：+Z 轴
					if (m_keyUp) localForce.setZ(localForce.z() - KEY_FORCE);
					if (m_keyDown) localForce.setZ(localForce.z() + KEY_FORCE);

					// 左：-X 轴；右：+X 轴
					if (m_keyLeft) localForce.setX(localForce.x() - KEY_FORCE);
					if (m_keyRight) localForce.setX(localForce.x() + KEY_FORCE);

					// 局部力转世界力
					btVector3 worldForce = basis * localForce;

					// 在质心施加全局坐标系轴向力
					body->applyCentralForce(worldForce);  //applyCentralForce 只接收世界坐标系力向量
				}
			}

			m_dynamicsWorld->stepSimulation(deltaTime);
		}
	}

	virtual void physicsDebugDraw(int debugFlags)
	{
		if (m_dynamicsWorld && m_dynamicsWorld->getDebugDrawer())
		{
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugFlags);
			m_dynamicsWorld->debugDrawWorld();
		}
	}

	virtual void exitPhysics()
	{
		removePickingConstraint();
		//cleanup in the reverse order of creation/initialization

		//remove the rigidbodies from the dynamics world and delete them

		if (m_dynamicsWorld)
		{
			int i;
			for (i = m_dynamicsWorld->getNumConstraints() - 1; i >= 0; i--)
			{
				m_dynamicsWorld->removeConstraint(m_dynamicsWorld->getConstraint(i));
			}
			for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
			{
				btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
				btRigidBody* body = btRigidBody::upcast(obj);
				if (body && body->getMotionState())
				{
					delete body->getMotionState();
				}
				m_dynamicsWorld->removeCollisionObject(obj);
				delete obj;
			}
		}
		//delete collision shapes
		for (int j = 0; j < m_collisionShapes.size(); j++)
		{
			btCollisionShape* shape = m_collisionShapes[j];
			delete shape;
		}
		m_collisionShapes.clear();

		delete m_dynamicsWorld;
		m_dynamicsWorld = 0;

		delete m_solver;
		m_solver = 0;

		delete m_broadphase;
		m_broadphase = 0;

		delete m_dispatcher;
		m_dispatcher = 0;

		delete m_collisionConfiguration;
		m_collisionConfiguration = 0;
	}

	virtual void debugDraw(int debugDrawFlags)
	{
		if (m_dynamicsWorld)
		{
			if (m_dynamicsWorld->getDebugDrawer())
			{
				m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugDrawFlags);
			}
			m_dynamicsWorld->debugDrawWorld();
		}
	}

	virtual bool keyboardCallback(int key, int state)
	{
		if ((key == B3G_F3) && state && m_dynamicsWorld)
		{
			btDefaultSerializer* serializer = new btDefaultSerializer();
			m_dynamicsWorld->serialize(serializer);

			FILE* file = fopen("testFile.bullet", "wb");
			fwrite(serializer->getBufferPointer(), serializer->getCurrentBufferSize(), 1, file);
			fclose(file);
			//b3Printf("btDefaultSerializer wrote testFile.bullet");
			delete serializer;
			return true;
		}
		// 260616FHP
		// handle B3G_LEFT_ARROW... apply centerforce on m_pickedBodyOnce
		bool pressed = (state == 1 /* GLFW_PRESS*/);
		switch (key)
		{
			case B3G_UP_ARROW:
				m_keyUp = pressed;
				break;
			case B3G_DOWN_ARROW:
				m_keyDown = pressed;
				break;
			case B3G_LEFT_ARROW:
				m_keyLeft = pressed;
				break;
			case B3G_RIGHT_ARROW:
				m_keyRight = pressed;
				break;
			// 可选：ESC清空选中
			case B3G_ESCAPE:
				if (m_pickedBodyOnce)
					m_pickedBodyOnce->forceActivationState(m_savedState);
					m_pickedBodyOnce->activate();
					m_pickedBodyOnce = nullptr;
				break;
		}
		return false;  //don't handle this key
	}

	btVector3 getRayTo(int x, int y)
	{
		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();

		if (!renderer)
		{
			btAssert(0);
			return btVector3(0, 0, 0);
		}

		float top = 1.f;
		float bottom = -1.f;
		float nearPlane = 1.f;
		float tanFov = (top - bottom) * 0.5f / nearPlane;
		float fov = btScalar(2.0) * btAtan(tanFov);

		btVector3 camPos, camTarget;

		renderer->getActiveCamera()->getCameraPosition(camPos);
		renderer->getActiveCamera()->getCameraTargetPosition(camTarget);

		btVector3 rayFrom = camPos;
		btVector3 rayForward = (camTarget - camPos);
		rayForward.normalize();
		float farPlane = 10000.f;
		rayForward *= farPlane;

		btVector3 rightOffset;
		btVector3 cameraUp = btVector3(0, 0, 0);
		cameraUp[m_guiHelper->getAppInterface()->getUpAxis()] = 1;

		btVector3 vertical = cameraUp;

		btVector3 hor;
		hor = rayForward.cross(vertical);
		hor.safeNormalize();
		vertical = hor.cross(rayForward);
		vertical.safeNormalize();

		float tanfov = tanf(0.5f * fov);

		hor *= 2.f * farPlane * tanfov;
		vertical *= 2.f * farPlane * tanfov;

		btScalar aspect;
		float width = float(renderer->getScreenWidth());
		float height = float(renderer->getScreenHeight());

		aspect = width / height;

		hor *= aspect;

		btVector3 rayToCenter = rayFrom + rayForward;
		btVector3 dHor = hor * 1.f / width;
		btVector3 dVert = vertical * 1.f / height;

		btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
		rayTo += btScalar(x) * dHor;
		rayTo -= btScalar(y) * dVert;
		return rayTo;
	}

	virtual bool mouseMoveCallback(float x, float y)
	{
		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();

		if (!renderer)
		{
			btAssert(0);
			return false;
		}

		btVector3 rayTo = getRayTo(int(x), int(y));
		btVector3 rayFrom;
		renderer->getActiveCamera()->getCameraPosition(rayFrom);
		movePickedBody(rayFrom, rayTo);

		return false;
	}

	virtual bool mouseButtonCallback(int button, int state, float x, float y)
	{
		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();

		if (!renderer)
		{
			btAssert(0);
			return false;
		}

		CommonWindowInterface* window = m_guiHelper->getAppInterface()->m_window;

#if 0
		if (window->isModifierKeyPressed(B3G_ALT))
		{
			printf("ALT pressed\n");
		} else
		{
			printf("NO ALT pressed\n");
		}
		
		if (window->isModifierKeyPressed(B3G_SHIFT))
		{
			printf("SHIFT pressed\n");
		} else
		{
			printf("NO SHIFT pressed\n");
		}
		
		if (window->isModifierKeyPressed(B3G_CONTROL))
		{
			printf("CONTROL pressed\n");
		} else
		{
			printf("NO CONTROL pressed\n");
		}
#endif

		if (state == 1)
		{
			if (button == 0 && (!window->isModifierKeyPressed(B3G_ALT) && !window->isModifierKeyPressed(B3G_CONTROL)))
			{
				btVector3 camPos;
				renderer->getActiveCamera()->getCameraPosition(camPos);

				btVector3 rayFrom = camPos;
				btVector3 rayTo = getRayTo(int(x), int(y));

				pickBody(rayFrom, rayTo);
			}
		}
		else
		{
			if (button == 0)
			{
				removePickingConstraint();
				//remove p2p
			}
		}

		//printf("button=%d, state=%d\n",button,state);
		return false;
	}

	virtual bool pickBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
	{
		if (m_dynamicsWorld == 0)
			return false;

		btCollisionWorld::ClosestRayResultCallback rayCallback(rayFromWorld, rayToWorld);

		rayCallback.m_flags |= btTriangleRaycastCallback::kF_UseGjkConvexCastRaytest;
		m_dynamicsWorld->rayTest(rayFromWorld, rayToWorld, rayCallback);
		if (rayCallback.hasHit())
		{
			btVector3 pickPos = rayCallback.m_hitPointWorld;
			btRigidBody* body = (btRigidBody*)btRigidBody::upcast(rayCallback.m_collisionObject);
			if (body)
			{
				//other exclusions?
				if (!(body->isStaticObject() || body->isKinematicObject()))
				{
					m_pickedBodyOnce = body;
					m_savedState = m_pickedBodyOnce->getActivationState();
					m_pickedBodyOnce->setActivationState(DISABLE_DEACTIVATION);
					#if FORBID_DRAG_CONSTRAINT
					m_pickedBody = body;
					m_savedState = m_pickedBody->getActivationState();
					m_pickedBody->setActivationState(DISABLE_DEACTIVATION);
					//printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());
					btVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;
					btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body, localPivot);
					m_dynamicsWorld->addConstraint(p2p, true);
					m_pickedConstraint = p2p;
					btScalar mousePickClamping = 30.f;
					p2p->m_setting.m_impulseClamp = mousePickClamping;
					//very weak constraint for picking
					p2p->m_setting.m_tau = 0.001f;
					#endif
				}
			}

			//					pickObject(pickPos, rayCallback.m_collisionObject);
			m_oldPickingPos = rayToWorld;
			m_hitPos = pickPos;
			m_oldPickingDist = (pickPos - rayFromWorld).length();
			//					printf("hit !\n");
			//add p2p
		}
		return false;
	}

	virtual bool monopolyKeyboardEvent()
	{
		// monopoly keyboard event when picked body, let keyboard signal broadcast to keyboardCallback only
		if (m_pickedBodyOnce)
			return true;
		return false;
	}

	virtual bool movePickedBody(const btVector3& rayFromWorld, const btVector3& rayToWorld)
	{
		if (m_pickedBody && m_pickedConstraint)
		{
			btPoint2PointConstraint* pickCon = static_cast<btPoint2PointConstraint*>(m_pickedConstraint);
			if (pickCon)
			{
				//keep it at the same picking distance

				btVector3 newPivotB;

				btVector3 dir = rayToWorld - rayFromWorld;
				dir.normalize();
				dir *= m_oldPickingDist;

				newPivotB = rayFromWorld + dir;
				pickCon->setPivotB(newPivotB);
				return true;
			}
		}
		return false;
	}
	virtual void removePickingConstraint()
	{
		if (m_pickedConstraint)
		{
			//m_pickedBody->forceActivationState(m_savedState);	// 260616FHP: move to B3G_ESCAPE keybaord callback
			//m_pickedBody->activate();
			m_dynamicsWorld->removeConstraint(m_pickedConstraint);
			delete m_pickedConstraint;
			m_pickedConstraint = 0;
			m_pickedBody = 0;
		}
	}

	btBoxShape* createBoxShape(const btVector3& halfExtents)
	{
		btBoxShape* box = new btBoxShape(halfExtents);
		return box;
	}

	void deleteRigidBody(btRigidBody* body)
	{
		int graphicsUid = body->getUserIndex();
		m_guiHelper->removeGraphicsInstance(graphicsUid);

		m_dynamicsWorld->removeRigidBody(body);
		btMotionState* ms = body->getMotionState();
		delete body;
		delete ms;
	}

	btRigidBody* createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, const btVector4& color = btVector4(1, 0, 0, 1))
	{
		btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			shape->calculateLocalInertia(mass, localInertia);

			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

		btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

		btRigidBody* body = new btRigidBody(cInfo);
		//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
		btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
		body->setWorldTransform(startTransform);
#endif  //

		body->setUserIndex(-1);
		m_dynamicsWorld->addRigidBody(body);
		return body;
	}

	virtual void renderScene()
	{
		if (m_dynamicsWorld)
		{
			{
				m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);
			}

			{
				m_guiHelper->render(m_dynamicsWorld);
			}
		}
	}
};

#endif  //COMMON_RIGID_BODY_SETUP_H
