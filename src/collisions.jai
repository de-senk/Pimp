//depends on Collision_Detection module.

//another name ->  Collision_Constraint :: Collision_Constraint; //manifold is a collection of points (we need only 4 or less points for good stability for all shapes)
Collision_Constraint :: struct {
    A: *Body;
    B: *Body;
    bucket_locator: Bucket_Locator;

    //for immediate mode
    latest_frame_of_existence: u64;
    hash: u64;

    disabled: bool;



    contactsBuffer: [4]Contact_Info;
    contacts_count: int;

    //xpbd specific needed for rigidbodies
    lambda_cache: float;
    impulse_cache: float;
    prev_velocity_A: Vector3;
    prev_velocity_B: Vector3;
    prev_angular_velocity_B: Vector3;
    prev_angular_velocity_A: Vector3;
    linear_compliance := FLOAT32_EPSILON4;
}


get_effective_mass :: (using body: *Body, normalized_world_force: Vector3, world_attach_position: Vector3) -> w: float
{
    //linear term
    w_linear := body.inv_mass;

    //angular term
    center_of_mass := body.position;
    world_lever := world_attach_position - center_of_mass;
    world_angular_force_direction := cross_product(world_lever, normalized_world_force);
    w_angular := get_effective_mass_angular_only(body, world_angular_force_direction);

    //total effective mass
    return w_linear + w_angular;
}




SolvePositions:: (using colli: *Collision_Constraint, dt: float) {

    for 0..contacts_count-1 {
        contact := contactsBuffer[it];
        contact_on_A := l2w(A.position, A.orientation, contact.local_point_on_A);
        contact_on_B := l2w(B.position, B.orientation, contact.local_point_on_B);
        normal := contact.world_normal_towards_A;

        penetration := dot(contact_on_A - contact_on_B, normal);
        if (penetration > 0.0) then continue;

        normal = normal;
        C := -penetration;
        basic.assert(is_normalized(normal));
        w1 := get_effective_mass(A, normal, contact_on_A); //(eq2)
        w2 := get_effective_mass(B, normal, contact_on_B); //(eq3)

       dt_squared := dt*dt;
       alpha := linear_compliance / dt_squared;
       dlambda := (- C - alpha*lambda_cache) / (w1 + w2 + alpha);
       if (w1 + w2) == 0 then dlambda = 0.0;
       lambda_cache += dlambda;

        p := -dlambda * normal;
        //(eq4) lamda caching NOT DONE BECAUSE I ONLY ITERATE ONCE.
        A.position += p*A.inv_mass; //(eq6)
        B.position -= p*B.inv_mass; //(eq7)

        #if ENABLE_ANGULAR_RESOLUTION {
            center_of_mass := A.position;
            world_leverA := contact_on_A - center_of_mass;
            world_leverB := contact_on_B - center_of_mass;

            I_worldA, inv_I_worldA := A.I_world, A.inv_I_world;
            I_worldB, inv_I_worldB := B.I_world, B.inv_I_world;

             //(eq8)
            delta_angularA := inv_I_worldA*cross_product(world_leverA, p);
            dq_for_A := 0.5*Quaternion.{delta_angularA.x,delta_angularA.y,delta_angularA.z,0.0}*A.orientation;
            A.orientation = normalize(dq_for_A + A.orientation);

             //(eq9)
            delta_angularB := inv_I_worldB*cross_product(world_leverB, p);
            dq_for_B := 0.5*Quaternion.{delta_angularB.x,delta_angularB.y,delta_angularB.z,0.0}*B.orientation;
            B.orientation = normalize(B.orientation - dq_for_B);
        }
    }

}

#import "Print_Vars";
original_authors3124123 :: #string CPP

bool PositionBasedRigidBodyDynamics::velocitySolve_RigidBodyContactConstraint(
	const Real invMass0,							// inverse mass is zero if body is static
	const Vector3r &x0, 						// center of mass of body 0
	const Vector3r &v0,						// velocity of body 0
	const Matrix3r &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
	const Vector3r &omega0,					// angular velocity of body 0
	const Real invMass1,							// inverse mass is zero if body is static
	const Vector3r &x1, 						// center of mass of body 1
	const Vector3r &v1,						// velocity of body 1
	const Matrix3r &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
	const Vector3r &omega1,					// angular velocity of body 1
	const Real stiffness,							// stiffness parameter of penalty impulse
	const Real frictionCoeff,						// friction coefficient
	Real &sum_impulses,							// sum of all impulses
	Eigen::Matrix<Real, 3, 5, Eigen::DontAlign> &constraintInfo,		// precomputed contact info
	Vector3r &corr_v0, Vector3r &corr_omega0,
	Vector3r &corr_v1, Vector3r &corr_omega1)
{
	// constraintInfo contains
	// 0:	contact point in body 0 (global)
	// 1:	contact point in body 1 (global)
	// 2:	contact normal in body 1 (global)
	// 3:	contact tangent (global)
	// 0,4:  1.0 / normal^T * K * normal
	// 1,4: maximal impulse in tangent direction
	// 2,4: goal velocity in normal direction after collision

	if ((invMass0 == 0.0) && (invMass1 == 0.0))
		return false;

	const Vector3r &connector0 = constraintInfo.col(0);
	const Vector3r &connector1 = constraintInfo.col(1);
	const Vector3r &normal = constraintInfo.col(2);
	const Vector3r &tangent = constraintInfo.col(3);

	// 1.0 / normal^T * K * normal
	const Real nKn_inv = constraintInfo(0, 4);

	// penetration depth
	const Real d = normal.dot(connector0 - connector1);

	// maximal impulse in tangent direction
	const Real pMax = constraintInfo(1, 4);

	// goal velocity in normal direction after collision
	const Real goal_u_rel_n = constraintInfo(2, 4);

	const Vector3r r0 = connector0 - x0;
	const Vector3r r1 = connector1 - x1;

	const Vector3r u0 = v0 + omega0.cross(r0);
	const Vector3r u1 = v1 + omega1.cross(r1);

	const Vector3r u_rel = u0-u1;
	const Real u_rel_n = u_rel.dot(normal);
	const Real delta_u_reln = goal_u_rel_n - u_rel_n;

	Real correctionMagnitude = nKn_inv * delta_u_reln;

	if (correctionMagnitude < -sum_impulses)
		correctionMagnitude = -sum_impulses;

	// add penalty impulse to counteract penetration
	if (d < 0.0)
		correctionMagnitude -= stiffness * nKn_inv * d;


	Vector3r p(correctionMagnitude * normal);
	sum_impulses += correctionMagnitude;

	// dynamic friction
	const Real pn = p.dot(normal);
	if (frictionCoeff * pn > pMax)
		p -= pMax * tangent;
	else if (frictionCoeff * pn < -pMax)
		p += pMax * tangent;
	else
		p -= frictionCoeff * pn * tangent;

	if (invMass0 != 0.0)
	{
		corr_v0 = invMass0*p;
		corr_omega0 = inertiaInverseW0 * (r0.cross(p));
	}

	if (invMass1 != 0.0)
	{
		corr_v1 = -invMass1*p;
		corr_omega1 = inertiaInverseW1 * (r1.cross(-p));
	}

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::init_RigidBodyContactConstraint(
	const Real invMass0,							// inverse mass is zero if body is static
	const Vector3r &x0,						// center of mass of body 0
	const Vector3r &v0,						// velocity of body 0
	const Matrix3r &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
	const Quaternionr &q0,					// rotation of body 0
	const Vector3r &omega0,					// angular velocity of body 0
	const Real invMass1,			// inverse mass is zero if body is static
	const Vector3r &x1,						// center of mass of body 1
	const Vector3r &v1,						// velocity of body 1
	const Matrix3r &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
	const Quaternionr &q1,					// rotation of body 1
	const Vector3r &omega1,					// angular velocity of body 1
	const Vector3r &cp0,						// contact point of body 0
	const Vector3r &cp1,						// contact point of body 1
	const Vector3r &normal,					// contact normal in body 1
	const Real restitutionCoeff,					// coefficient of restitution
	Eigen::Matrix<Real, 3, 5, Eigen::DontAlign> &constraintInfo)
{
	// constraintInfo contains
	// 0:	contact point in body 0 (global)
	// 1:	contact point in body 1 (global)
	// 2:	contact normal in body 1 (global)
	// 3:	contact tangent (global)
	// 0,4:  1.0 / normal^T * K * normal
	// 1,4: maximal impulse in tangent direction
	// 2,4: goal velocity in normal direction after collision

	// compute goal velocity in normal direction after collision
	const Vector3r r0 = cp0 - x0;
	const Vector3r r1 = cp1 - x1;

	const Vector3r u0 = v0 + omega0.cross(r0);
	const Vector3r u1 = v1 + omega1.cross(r1);
	const Vector3r u_rel = u0 - u1;
	const Real u_rel_n = normal.dot(u_rel);

	constraintInfo.col(0) = cp0;
	constraintInfo.col(1) = cp1;
	constraintInfo.col(2) = normal;

	// tangent direction
	Vector3r t = u_rel - u_rel_n*normal;
	Real tl2 = t.squaredNorm();
	if (tl2 > 1.0e-6)
		t *= static_cast<Real>(1.0) / sqrt(tl2);

	constraintInfo.col(3) = t;

	// determine K matrix
	Matrix3r K1, K2;
	computeMatrixK(cp0, invMass0, x0, inertiaInverseW0, K1);
	computeMatrixK(cp1, invMass1, x1, inertiaInverseW1, K2);
	Matrix3r K = K1 + K2;

	constraintInfo(0, 4) = static_cast<Real>(1.0) / (normal.dot(K*normal));

	// maximal impulse in tangent direction
	constraintInfo(1, 4) = static_cast<Real>(1.0) / (t.dot(K*t)) * u_rel.dot(t);

	// goal velocity in normal direction after collision
	constraintInfo(2, 4) = 0.0;
	if (u_rel_n < 0.0)
		constraintInfo(2, 4) = -restitutionCoeff * u_rel_n;

	return true;
}

CPP;
#if 0 {
SolveVelocities :: (using colli: *Collision_Constraint, dt: float) {
    //parameters
	normal: Vector3 = colli.contactsBuffer[0].world_normal_towards_A;
    gravity_length := length(context.solver.gravity);
    h:= dt;

    //center of masses
    x0 := A.position;
    x1 := B.position;

    //velocities
    v0 := A.velocity;
    v1 := B.velocity;
    omega0 := A.angular_velocity;
    omega1 := B.angular_velocity;

    invMass0 := A.inv_mass;
    invMass1 := B.inv_mass;

    //I precompute these each frame
    I_worldA, inertiaInverseW0 := A.I_world, A.inv_I_world;
    I_worldB, inertiaInverseW1 := B.I_world, B.inv_I_world;
    stiffness := 0.0;
    frictionCoeff := 0.5;
    sum_impulses := 0.0;

    //to write to
	corr_v0: Vector3;
	corr_v1: Vector3;
	corr_omega0: Vector3;
	corr_omega1: Vector3;
    pMax := 100.0;
    restitutionCoeff := 1.0;
	//l2w stands for local to world
    for 0..colli.contacts_count-1 {
        c := colli.contactsBuffer[it];
        connector0 := l2w(A.position, A.orientation, c.local_point_on_A);
        connector1 := l2w(B.position, B.orientation, c.local_point_on_B);

        prev_contact_on_A := l2w(A.prev_position, A.prev_orientation, c.local_point_on_A);
        prev_contact_on_B := l2w(B.prev_position, B.prev_orientation, c.local_point_on_B);
        prev_penetration := dot(prev_contact_on_A - prev_contact_on_B, normal);
        prev_leverA := prev_contact_on_A - A.prev_position;
        prev_leverB := prev_contact_on_B - B.prev_position;

        // 1.0 / normal^T * K * normal
        // nKn_inv := constraintInfo(0, 4);
        w1 := get_effective_mass(A, normal, connector0);
        w2 := get_effective_mass(B, normal, connector1);
        nKn_inv := 1.0 / (w1+w2);

       // goal velocity in normal direction after collision
        goal_u_rel := (prev_velocity_A + cross(prev_angular_velocity_A, prev_leverA)) - (prev_velocity_B + cross(prev_angular_velocity_B, prev_leverB));

       // if (u_rel_n < 0.0)
       // 	  constraintInfo(2, 4) = -restitutionCoeff * u_rel_n;

	    // penetration depth
	    d := dot(connector0 - connector1, normal);

    	goal_u_rel_n := -restitutionCoeff*dot(normal, goal_u_rel);



    	r0 := connector0 - x0;
    	r1 := connector1 - x1;


    	u0 := v0 + cross(omega0, r0);
    	u1 := v1 + cross(omega1, r1);

	    u_rel := u0-u1;
        u_rel_n := dot(u_rel, normal);

        delta_u_reln := goal_u_rel_n - u_rel_n;


        correctionMagnitude := nKn_inv * delta_u_reln;

        if correctionMagnitude < -sum_impulses
		    then correctionMagnitude = -sum_impulses;


    	// add penalty impulse to counteract penetration
    	if d < 0.0
    		then correctionMagnitude -= stiffness * nKn_inv * d;


    	p := correctionMagnitude * normal;
    	sum_impulses += correctionMagnitude;

        draw_primitive.arrow(connector0, p);
        // dynamic friction
    	pn := dot(p, normal);


	   tangent := u_rel - u_rel_n*normal;
	   tl2 := length_squared(tangent);
	   if tl2 > FLOAT32_EPSILON6
		tangent *= 1.0 / sqrt(tl2);

    	if frictionCoeff * pn > pMax
    		p -= pMax * tangent;
    	else if frictionCoeff * pn < -pMax
    		p += pMax * tangent;
    	else
    		p -= frictionCoeff *  tangent;

		corr_v0 = invMass0*p;
		corr_omega0 = inertiaInverseW0 * cross(r0, p);

		corr_v1 = -invMass1*p;
		corr_omega1 = inertiaInverseW1 * cross(r1, -p);

    }

    A.velocity += corr_v0;
    B.velocity += corr_v1;
    A.angular_velocity += corr_omega0;
    B.angular_velocity += corr_omega1;

    basic.assert(is_valid(A.position), "your body exploded");
    basic.assert(is_valid(A.velocity), "your body tried to explode");

}
} else {


SolveVelocities :: (using colli: *Collision_Constraint, dt: float) {
    is_negative :: (b: float32) -> bool {return b < 0.0;};
	normal: Vector3 = colli.contactsBuffer[0].world_normal_towards_A;
    gravity_length := length(context.solver.gravity);
    h:= dt;
    //I precompute these each frame
    I_worldA, inv_I_worldA := A.I_world, A.inv_I_world;
    I_worldB, inv_I_worldB := B.I_world, B.inv_I_world;
    //l2w stands for local to world
    for 0..colli.contacts_count-1 {
        c := colli.contactsBuffer[it];
        contact_on_A := l2w(A.position, A.orientation, c.local_point_on_A);
        contact_on_B := l2w(B.position, B.orientation, c.local_point_on_B);
        penetration := dot(contact_on_A - contact_on_B, normal);
        leverA := contact_on_A - A.position;
        leverB := contact_on_B - B.position;

        rel_vel := (A.velocity + cross(A.angular_velocity, leverA)) - (B.velocity + cross(B.angular_velocity, leverB));
		rel_vel_comp_n := dot(rel_vel, normal); //component along normal
		PREV_rel_vel := (prev_velocity_A + cross(prev_angular_velocity_A, leverA)) - (prev_velocity_B + cross(prev_angular_velocity_B, leverB));
		PREV_rel_vel_comp_n := dot(normal, PREV_rel_vel);
		reflected_PREV_rel_vel_comp_n := PREV_rel_vel_comp_n;

		// tangent velocities not implemented yet.
    	u := basic.max(A.friction, B.friction); //0.5
       //normal points towards A
       // if (penetration > 0.0) then continue; //should we do this?
       already_separating := rel_vel_comp_n >= -0;
       if already_separating then continue;

		// normal velocities (restitution)
    	e := A.restitution * B.restitution; // 0.5 * 0.5
        if abs(rel_vel_comp_n) <= h*2.0*gravity_length
             then e = 0.0;

         //no e = 1.0;//checkin FORCED TO BOUNCE

        //(eq34)
        nullification := -rel_vel_comp_n;
        restitution := basic.max(-e * PREV_rel_vel_comp_n, 0.0);
        if PREV_rel_vel_comp_n <0 || PREV_rel_vel_comp_n > 0
        basic.assert(is_negative(nullification) == is_negative(restitution), "nullification should have the same sign as restitution nulli,resti % %", nullification, restitution);
        dv :=  normal * (restitution + nullification);
        w1 := get_effective_mass(A, normal, contact_on_A);
        w2 := get_effective_mass(B, normal, contact_on_B);
		//(eq33)
		p := dv / (w1 + w2);
        A.velocity += p * A.inv_mass;
        B.velocity -= p * B.inv_mass;
        A.angular_velocity += inv_I_worldA * cross(leverA, p);; //leverA = r1
        B.angular_velocity -= inv_I_worldB * cross(leverB, p);  //leverB = r2
    }

}


asdasd :: #string CPP
void SolveVelocities(Collision_Constraint* colli, float dt) {
    auto is_negative = [](float b) -> bool { return b < 0.0f; };
    Vector3 normal = colli->contactsBuffer[0].world_normal_towards_A;
    float gravity_length = length(context.solver.gravity);
    float h = dt;
    // Precomputed each frame
    Matrix3 I_worldA = colli->A->I_world;          Matrix3 I_worldB = colli->B->I_world;
    Matrix3 inv_I_worldA = colli->A->inv_I_world;  Matrix3 inv_I_worldB = colli->B->inv_I_world;

    // l2w stands for local to world
    for (int i = 0; i < colli->contacts_count; i++) {
        Contact c = colli->contactsBuffer[i];
        Vector3 contact_on_A = l2w(colli->A->position, colli->A->orientation, c.local_point_on_A);
        Vector3 contact_on_B = l2w(colli->B->position, colli->B->orientation, c.local_point_on_B);
        float penetration = dot(contact_on_A - contact_on_B, normal); // normal points towards A
        Vector3 leverA = contact_on_A - colli->A->position;
        Vector3 leverB = contact_on_B - colli->B->position;
        Vector3 rel_vel = (colli->A->velocity + cross(colli->A->angular_velocity, leverA))
                        - (colli->B->velocity + cross(colli->B->angular_velocity, leverB));
        float rel_vel_comp_n = dot(rel_vel, normal); // component along normal
        Vector3 PREV_rel_vel = (colli->prev_velocity_A + cross(colli->prev_angular_velocity_A, leverA)) //prev_ were stored beforehand
                             - (colli->prev_velocity_B + cross(colli->prev_angular_velocity_B, leverB));//prev_ were stored beforehand
        float PREV_rel_vel_comp_n = dot(normal, PREV_rel_vel);
        float u = max(colli->A->friction, colli->B->friction); // 0.5 //NOTE(tr): tangent velocities not implemented yet
        bool already_separating = rel_vel_comp_n >= 0.0f;
        if (already_separating) continue;
        // normal velocities (restitution)
        float e = colli->A->restitution * colli->B->restitution;
        if (abs(rel_vel_comp_n) <= h * 2.0f * gravity_length)
            e = 0.0f;
        e = 1.0f; // FORCED TO BOUNCE ! !!!!!!!!!!!!!!!

        // (eq34)
        float nullification = -rel_vel_comp_n;
        float restitution = max(-e * PREV_rel_vel_comp_n, 0.0f);
        if (PREV_rel_vel_comp_n < 0 || PREV_rel_vel_comp_n > 0)  assert(is_negative(nullification) == is_negative(restitution));
        Vector3 dv = normal * (restitution + nullification);
        float w1 = get_effective_mass(colli->A, normal, contact_on_A);  float w2 = get_effective_mass(colli->B, normal, contact_on_B);
        // (eq33)
        Vector3 p = dv / (w1 + w2);
        colli->A->velocity += p * colli->A->inv_mass;                   colli->B->velocity -= p * colli->B->inv_mass;
        colli->A->angular_velocity += inv_I_worldA * cross(leverA, p);  colli->B->angular_velocity -= inv_I_worldB * cross(leverB, p);
    }
}
CPP;

SolveVelocitiesOLD :: (using colli: *Collision_Constraint, dt: float) {
    is_negative :: (b: float32) -> bool {return b < 0.0;};
	normal: Vector3 = colli.contactsBuffer[0].world_normal_towards_A;
    gravity_length := length(context.solver.gravity);
    h:= dt;

    //I precompute these each frame
    I_worldA, inv_I_worldA := A.I_world, A.inv_I_world;
    I_worldB, inv_I_worldB := B.I_world, B.inv_I_world;
    sum_linear_momentum_on_A: Vector3;
    sum_angular_momentum_on_A: Vector3;
    sum_angular_momentum_on_B: Vector3;
    //l2w stands for local to world
    for 0..colli.contacts_count-1 {
        c := colli.contactsBuffer[it];
        contact_on_A := l2w(A.position, A.orientation, c.local_point_on_A);
        contact_on_B := l2w(B.position, B.orientation, c.local_point_on_B);
        penetration := dot(contact_on_A - contact_on_B, normal);
        leverA := contact_on_A - A.position;
        leverB := contact_on_B - B.position;

        prev_contact_on_A := l2w(A.prev_position, A.prev_orientation, c.local_point_on_A);
        prev_contact_on_B := l2w(B.prev_position, B.prev_orientation, c.local_point_on_B);
        prev_penetration := dot(prev_contact_on_A - prev_contact_on_B, normal);
        prev_leverA := prev_contact_on_A - A.prev_position;
        prev_leverB := prev_contact_on_B - B.prev_position;




        rel_vel := (A.velocity + cross(A.angular_velocity, leverA)) - (B.velocity + cross(B.angular_velocity, leverB));
		rel_vel_comp_n := dot(rel_vel, normal); //component along normal
		PREV_rel_vel := (prev_velocity_A + cross(prev_angular_velocity_A, leverA)) - (prev_velocity_B + cross(prev_angular_velocity_B, leverB));
		PREV_rel_vel_comp_n := dot(normal, PREV_rel_vel);
		reflected_PREV_rel_vel_comp_n := PREV_rel_vel_comp_n;

        // if already_separating then continue;
		// tangent velocities not implemented yet.
    	u := basic.max(A.friction, B.friction); //0.5


        //normal points towards A
       // if (penetration > 0.0) then continue; //should we do this?
       already_separating := rel_vel_comp_n >= -0;
       // if already_separating then rel_vel_comp_n = -0.000001;
       if already_separating then continue;

		// normal velocities (restitution)
    	e := A.restitution * B.restitution; // 0.5 * 0.5
        if abs(rel_vel_comp_n) <= h*2.0*gravity_length
             then e = 0.0;

        e = 1.0; //FORCED TO BOUNCE

        //(eq34)
        nullification := -rel_vel_comp_n;
        restitution := basic.max(-e * PREV_rel_vel_comp_n, 0.0);
        if PREV_rel_vel_comp_n <0 || PREV_rel_vel_comp_n > 0
        log_vars(restitution);
        log_vars(nullification);
        basic.assert(is_negative(nullification) == is_negative(restitution), "nullification should have the same sign as restitution nulli,resti % %", nullification, restitution);
        dv :=  normal * (restitution + nullification);

        w1 := get_effective_mass(A, normal, contact_on_A);
        w2 := get_effective_mass(B, normal, contact_on_B);

		//(eq33)
		p := dv / (w1 + w2);

        sum_linear_momentum_on_A += p;
        sum_angular_momentum_on_A += cross(leverA, p);
        sum_angular_momentum_on_B += cross(leverB, p); //positive because we use - later on
        A.velocity += p * A.inv_mass;
        B.velocity -= p * B.inv_mass;
        A.angular_velocity += inv_I_worldA * cross(leverA, p);; //leverA = r1
        B.angular_velocity -= inv_I_worldB * cross(leverB, p);  //leverB = r2

        //maybe here
        // A.prev_velocity += A.inv_mass*p;
        // A.prev_angular_velocity = inv_I_worldB*cross(leverA, p);
        // A.prev_velocity = A.velocity;
        // A.prev_angular_velocity = A.angular_velocity;
        // B.prev_velocity = B.velocity;
        // B.prev_angular_velocity = B.angular_velocity;
        // prev_velocity_A = A.velocity;
        // prev_velocity_B = B.velocity;
        // prev_angular_velocity_A = A.angular_velocity;
        // prev_angular_velocity_B = B.angular_velocity;
    }
     // A.velocity +=  A.inv_mass * sum_linear_momentum_on_A;
     // B.velocity -=  B.inv_mass * sum_linear_momentum_on_A;
     // A.angular_velocity += inv_I_worldA * sum_angular_momentum_on_A;
     // B.angular_velocity -= inv_I_worldB * sum_angular_momentum_on_B;

    basic.assert(is_valid(A.position), "your body exploded");
    basic.assert(is_valid(A.velocity), "your body tried to explode");

}
}

SolveJointPositions :: (joint: *Joint_Constraint, dt: float) {
    //solve(it, dt); FLATTENED OUT:
    //these are callbacks of procedures stored inside each joint, I think this strategy is better than a switch statement.
    joint.pre_solve(joint, dt);
    recalculate_world_attach_pos_and_ori(joint);
    joint.solve_position(joint, dt);
    recalculate_world_attach_pos_and_ori(joint); //maybe I need one of these here?
    joint.solve_orientation(joint, dt);
}

ApplyDamping :: (joint: *Joint_Constraint, dt: float) {
     recalculate_world_attach_pos_and_ori(joint); //maybe I need one of these here?
    joint.apply_linear_damping(joint, dt);
     recalculate_world_attach_pos_and_ori(joint); //maybe I need one of these here?
    joint.apply_angular_damping(joint, dt);
}

AddManifold :: ( A: *Body, B: *Body, contact_stack_lifetime: []Contact_Info) -> *Collision_Constraint {
    using context.solver;
    #if DEBUG basic.assert(contact_stack_lifetime.count != 0, "You tried to create a manifold with 0 contact points, is that ok?");
    locator:, manifold := bucket_array_add(*context.solver.collision_constraints,.{});
    manifold.bucket_locator = locator;

    manifold.A = A;
    manifold.B = B;


    for it_index: 0..3
    {
        // manifold.contactsBuffer[it_index].A = A; //<-this is the hotfix :tagOtherwiseWeNullDerefHere
        // manifold.contactsBuffer[it_index].B = B; //<-this is the hotfix :tagOtherwiseWeNullDerefHere
    }
    UpdateManifold(manifold, contact_stack_lifetime);
    //hmm, well we should never be able to "grab/select" penetration constraints..
    // if A array_add(*manifold.A.local_constraints, manifold);
    // if B array_add(*manifold.B.local_constraints, manifold);
    // // manifold.local_normal_A = 2*normal;


    return manifold;
}


UpdateManifold :: (using manifold: *Collision_Constraint, contact_stack_lifetime: []Contact_Info) {

for contact_stack_lifetime {
        #if DEBUG basic.assert(is_normalized(it.world_normal_towards_A , tolerance=FLOAT32_EPSILON5), "whaat  %", it.world_normal_towards_A );
        manifold.contactsBuffer[it_index] = it;
        // manifold.contactsBuffer[it_index].world_contact_point_on_A = it.world_contact_point_on_A;
        // manifold.contactsBuffer[it_index].world_contact_point_on_B = it.world_contact_point_on_B;
        // manifold.contactsBuffer[it_index].world_normal_towards_A = it.world_normal_towards_A;
    }
    manifold.contacts_count = contact_stack_lifetime.count;
}


broad_phase :: (broad_strategy: enum{SWEEP_AND_PRUNE;}) { // grep BroadPhase ::
    using context.solver;
    basic.array_reset_keeping_memory(*possible_collisions); //we reset here so we can still draw them when we stop time
    basic.array_reset_keeping_memory(*collision_pairs); //we reset here so we can stop time and draw these without problems
    //stragety 1
    SweepAndPrune1D(*collision_pairs);
}


SweepAndPrune1D :: (final_pairs: *[..]Collision_Pair ) {
    using context.solver;
    sorted_bodies: [..]Pseudo_Body;
    sorted_bodies.allocator = basic.temp;
    worst_case_count := bodies.count*2 /* mul by 2 because pairs! */;
    maybe_reserve(*sorted_bodies, worst_case_count);

    //sort the mfs
    {
        axis_to_project_aabbs := #run normalize(Vector3.{1,1,1});
        for * bodies {
            aabb_copy := it.aabb;
            #if DEBUG basic.assert(is_aabb_valid(aabb_copy), "aabb is invalid %", aabb_copy);
            maybe_expand_aabb(*aabb_copy, aabb_copy.min + it.velocity*dt);
            maybe_expand_aabb(*aabb_copy, aabb_copy.max + it.velocity*dt);
            maybe_expand_aabb(*aabb_copy, aabb_copy.min - COLLISION_MARGIN*axis_to_project_aabbs); //I think using the same collision margin for everything will get me in trouble in the future.
            maybe_expand_aabb(*aabb_copy, aabb_copy.max + COLLISION_MARGIN*axis_to_project_aabbs);

            pA := basic.array_add(*sorted_bodies);
            pA.bucket_locator = it.bucket_locator;
            pA.value = dot(aabb_copy.min, axis_to_project_aabbs);
            pA.ismin = true;

            pB := basic.array_add(*sorted_bodies);
            pB.bucket_locator = it.bucket_locator;
            pB.value = dot(aabb_copy.max, axis_to_project_aabbs);
            pB.ismin = false;

            // sorted_bodies[it_index*2 + 0].bucket_locator = it.bucket_locator;
            // sorted_bodies[it_index*2 + 0].value = dot(aabb_copy.min, axis_to_project_aabbs);
            // sorted_bodies[it_index*2 + 0].ismin = true;
            // sorted_bodies[it_index*2 + 1].bucket_locator = it.bucket_locator;
            // sorted_bodies[it_index*2 + 1].value = dot(aabb_copy.max, axis_to_project_aabbs);
            // sorted_bodies[it_index*2 + 1].ismin = false;

        }
        quick_sort(sorted_bodies, CompareSAP);
    }

    //pair the mfs
    {
        basic.array_reset_keeping_memory(final_pairs);
        for a: sorted_bodies
        {
            if !a.ismin then continue;
            pair: Collision_Pair;
            pair.a = a.bucket_locator;

            //TODO: rewrite all this crap and I hate using a b everywhere
            for j: it_index + 1 .. sorted_bodies.count - 1
            {
                b := sorted_bodies[j];
                if b.bucket_locator == a.bucket_locator then break;
                if !b.ismin then continue;
                pair.b = b.bucket_locator;
                basic.array_add(final_pairs, pair);
            }
        }
    }
}

narrow_phase :: ()
{
    already_constrained_pair :: (A: *Body, B: *Body) -> bool
    {
        // add_to_array_of_contact_points((A.position), radii = length(A.size)/2);
        // add_to_array_of_contact_points((B.position), radii = length(B.size)/2);
        for context.solver.collision_constraints
        {
            // if it.kind == .IGNORE_COLLISION then return true;
            are_a_and_b_constrained := (it.A == A && it.B == B) || (it.A == B && it.B == A);
            if are_a_and_b_constrained
            // && it.kind == .COLLISION
            {
             return true;
            }

        }
        return false;
    }

    using context.solver;
    for collision_pairs
    {
        A :*Body= bucket_array_find_pointer(*bodies, it.a);
        B :          = bucket_array_find_pointer(*bodies, it.b);


        #if DEBUG basic.assert(A != null);
        #if DEBUG basic.assert(B != null);

        should_swap_order_by_shape := A.shape_kind > B.shape_kind;

        are_same_shape := A.shape_kind == B.shape_kind;
        if are_same_shape then should_swap_order_by_shape = A.(u64) < B.(u64);
        if should_swap_order_by_shape  A, B = B,A;

        //maybe can be abstracted into a procedure, compressed* into a procedure, the problem is finding a good name for this
        {
            if A.inv_mass == 0 && B.inv_mass == 0 then continue; //otherwise we trigger bad collisions and crash stuff

            old_manifold: *Collision_Constraint; //pointer, why not?
            are_already_constrained: bool;
            {////WRONG ASSUMPTION, what I was thinking? 5+5 == 4+6 == 3 + 7 == 10 ---> WRONG ---> maybe make a hash table pointer A + pointer B as keys, since A + B == B + A, we remove duplicates for free

                //@Slow read the above comments for an idea in how to optimize this
                for *collision_constraints
                {
                    // log("are_already_constrained % -- %", it.A.(u64) + it.B.(u64), A.(u64) + B.(u64));
                    c1 := it.A == A && it.B == B;
                    c2 := it.A == B && it.B == A;
                    if c1 || c2
                    {
                        // log("are_already_constrained");
                        are_already_constrained = true;
                        old_manifold = it;
                        #if DEBUG basic.assert(it.contacts_count != 0, "% how we have allocated a manifold with no contacts? you need to check all manifolds before this and remove the ones with no contacts or something", it.contacts_count);

                    }
                }

            }
            new_contacts_count, new_contacts := compute_contacts(A, B); //@Maybe should we early return false?

            // are_already_constrained := already_constrained_pair(A,B);


            should_try_create_manifold := !are_already_constrained;
            should_compare_with_old    := are_already_constrained;

            //compare to old contacts, and keep the data if they satisfy some conditions
            if should_compare_with_old
            {
                //conditions to hold:
                //1) previous contact pos tangential difference is smaller than threshold
                //2)

                //idk man
                if new_contacts_count != old_manifold.contacts_count
                {
                    //TODO:
                }

                    // if new_contacts_count >= old_manifold.contacts_count
                    {

                    used := bool.[false, false,false,false];
                    for new_idx: 0.. new_contacts_count - 1
                    {
                        if new_contacts[new_idx].feature.value == 0 then basic.log("this is BAD, you are not creating an local UID(feature.value and feature.kind) for contact points! %\n ",old_manifold.contactsBuffer[new_idx].feature );
                        for old_jdx: 0.. old_manifold.contacts_count - 1
                        {
                            if used[old_jdx] == true then continue;
                            is_same_feature := new_contacts[new_idx].feature.value == old_manifold.contactsBuffer[old_jdx].feature.value;
                            if is_same_feature
                            {
                                old_pointA := old_manifold.contactsBuffer[old_jdx].local_point_on_A;
                                old_pointB := old_manifold.contactsBuffer[old_jdx].local_point_on_B;
                                new_pointA := new_contacts[old_jdx].local_point_on_A;
                                new_pointB := new_contacts[old_jdx].local_point_on_B;

                                l1 := length_squared(new_pointA - old_pointA);
                                l2 := length_squared(new_pointB - old_pointB);
                                contacts_are_too_far := l1 > 0.5 || l2 > 0.5;
                                if !contacts_are_too_far
                                {
                                    used[old_jdx] = true;
                                }

                               //TODO(tr): STATICK FRICTION / DYNAMIC FRICTION

                            }
                        }

                    // old_manifold.contacts_count = new_contacts_count;
                    old_manifold.contacts_count = 0;
                    for new_idx: 0.. new_contacts_count - 1
                    {
                        if used[new_idx]
                        {
                            old_manifold.contacts_count += 1;
                        }else {
                            old_manifold.contacts_count += 1;
                          // old_manifold.contactsBuffer[new_idx] = new_contacts[new_idx];
                        }
                    }



                    }

                }
                //@dontdelete
                //sanity @speed but this is super fast anyways @hack too, meh I h should comment this and dont delete the comments and use the tag to grep.
                // for 0..new_contacts_count-1 {
                //             old_manifold.contactsBuffer[idx].A = A; //:tagOtherwiseWeNullDerefHere
                //             old_manifold.contactsBuffer[idx].B = B; //:tagOtherwiseWeNullDerefHere
                // }

                // old_manifold.contactsBuffer = new_contacts;
                {

                contact_stack_lifetime: []Contact_Info = new_contacts;
                    cter := 0;
                     for 0..new_contacts_count-1
                        {

                            contact := new_contacts[it];
                            if !is_normalized(contact.world_normal_towards_A) then continue; //TODO: figure out why the first contact in history is normal 0
                            cter += 1;
                            #if DEBUG basic.assert(is_normalized(contact.world_normal_towards_A , tolerance=FLOAT32_EPSILON5), "weeaat  %", contact.world_normal_towards_A );

                            //for debugging purpusese you can use this: instead of addmanifold
                            // AddPenetration(contact.A, contact.B, contact.local_point_A, contact.local_point_B, contact.world_normal_towards_A);
                        }
                    contact_stack_lifetime.count = cter;
                    UpdateManifold(old_manifold, contact_stack_lifetime);
                }
                for it_index: 0..new_contacts_count -1
    //             #if DEBUG assert(is_normalized(old_manifold.contactsBuffer[it_index].local_normal_A , tolerance=FLOAT32_EPSILON5), "aaa  %", old_manifold.contactsBuffer[it_index].local_normal_A );
                for it_index: 0..new_contacts_count -1 {
                #if DEBUG basic.assert(is_normalized(new_contacts[it_index].world_normal_towards_A , tolerance=FLOAT32_EPSILON5), "ccc  %", new_contacts[it_index].world_normal_towards_A );
    //             #if DEBUG assert(is_normalized(old_manifold.contactsBuffer[it_index].local_normal_A , tolerance=FLOAT32_EPSILON5), "ccc  %", old_manifold.contactsBuffer[it_index].local_normal_A );
                }


            } //else

            if should_try_create_manifold
            {
                found_a_ballistic_contact: bool;
                best_ballistic_contact_index: int;
                best_ballistic_contact_toi: float = FLOAT32_INFINITY;;
                for c_idx:0..new_contacts_count-1
                {
                    contact := new_contacts[c_idx];
                    if contact.time_of_impact > 0.0
                    {
                        // #if DEBUG assert(contact.A != null);
                        // #if DEBUG assert(contact.B != null);

                        found_a_ballistic_contact = true;
                        if contact.time_of_impact < best_ballistic_contact_toi
                        {
                            best_ballistic_contact_toi = contact.time_of_impact;
                            best_ballistic_contact_index = c_idx;

                        }

                    }
                }

                if found_a_ballistic_contact
                {
                    best_contact := new_contacts[best_ballistic_contact_index];
                    basic.array_add(*context.solver.possible_collisions, best_contact); //add to CCD list (ballisitc solver) //maybe use arena instead because this is reseted per frame//so we basically save a backup of the computed data to check if between 0 and dt we have a collision.
                    // log("BALLISIT CONTACT ADDED");
                }
                else
                {
                    contact_stack_lifetime:[]Contact_Info = new_contacts;
                    cter := 0;
                    for 0..new_contacts_count-1
                    {
                        contact := new_contacts[it];
                        if !is_normalized(contact.world_normal_towards_A) then continue; //TODO: figure out why the first contact in history is normal 0
                        cter += 1;
                        //for debugging purpusese you can use this: instead of addmanifold
                        // AddPenetration(contact.A, contact.B, contact.local_point_A, contact.local_point_B, contact.world_normal_towards_A);
                    }
                    contact_stack_lifetime_view: []Contact_Info = contact_stack_lifetime;;
                    contact_stack_lifetime_view.count = cter;
                    //[IMPORTANT] NOTE: when debugging, if you are not removing contacts, and you iterated with 0 contacts, then above booleans will trigger and this code gets executed once, why im writtin this? because I wasted 5 minutes of my life not knowing why contacts suddenly stopped working
                    if cter != 0 {

                        ptr_manifold := AddManifold(A, B, contact_stack_lifetime_view);
                        #if DEBUG basic.assert(ptr_manifold.contacts_count != 0, "GOW?????????");

                        }
                }
            } //if !should_compare_with_old end
        } //block block



    } //for collision pairs


}


CollisionDetection :: () {
  using context.solver;
  for *bodies { //@Hack per frame caclulate inverse of all hulls
      if it.shape_kind == .HULL {
          it.hull_shape.worldSpace = get_composed_transform(it.position, it.orientation, it.scale);
          it.hull_shape.inv_worldSpace_for_bivectors = calculate_inverse_or_use_transpose(it.hull_shape);
      }
  }
  //broad and narrow phase, I dont like how I implemented this entire block
  broad_phase(.SWEEP_AND_PRUNE); //AABB shenanigans TODO: Trees and Octopuses (BVH BHV is the good one)
  maybe_reserve(*possible_collisions, collision_pairs.count, 1.2); //broad phase may have increased that number. so we reserve memory.
  narrow_phase(); //here we run SAT and compare old contacts with new contacts
}

/////////////////
//Im not happy about these  but whatever for now.
fill_contact_data_for_wraps :: (
A: *Body, B: *Body,
    contact_data: *Contact_Info,
    penetration: float,
    world_normal_towards_A: Vector3,
    world_midpoint: Vector3,
    world_contact_point_on_A: Vector3,
    world_contact_point_on_B: Vector3
) {
    contact_data.penetration = penetration;
    contact_data.world_midpoint = world_midpoint;
    contact_data.local_point_on_A = w2l( A.position, A.orientation, world_contact_point_on_A);
    contact_data.local_point_on_B = w2l( B.position, B.orientation, world_contact_point_on_B);
    contact_data.world_normal_towards_A = world_normal_towards_A;

}
wrap_collide_sphere_vs_sphere :: (A: *Body, B: *Body) -> contacts_count:int, contacts_data: [4]Contact_Info{
    num_contacts,
    penetration,
    world_normal_towards_A,
    world_midpoint,
    world_contact_point_on_A,
    world_contact_point_on_B := collide_sphere_vs_sphere(A.position, A.radius, B.position, B.radius);

    //setting up return data
    contacts_count:=num_contacts;
    contacts_data : [4]Contact_Info;
    for 0..contacts_count-1 {
        fill_contact_data_for_wraps(A,B, *contacts_data[0],penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B);
    }
    return contacts_count, contacts_data;

}
wrap_collide_sphere_vs_capsule:: (A: *Body, B: *Body) -> contacts_count:int, contacts_data: [4]Contact_Info{
    capsule_start_B := B.position;
    capsule_end_B := capsule_start_B + B.capsule_height*rotate(Vector3.{0,1,0}, B.orientation);

    num_contacts, penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B
    := collide_sphere_vs_capsule(A.position, A.radius, capsule_start_B ,capsule_end_B, B.radius);

    contacts_count:=num_contacts;
    contacts_data: [4]Contact_Info;

    for 0..contacts_count-1 {
    fill_contact_data_for_wraps(A,B, *contacts_data[it],penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B);
    }

    return contacts_count, contacts_data;
}
wrap_collide_capsule_vs_capsule:: (A: *Body, B: *Body) -> contacts_count:int, contacts_data: [4]Contact_Info{
    capsule_start_A := A.position;
    capsule_end_A := capsule_start_A + A.capsule_height*rotate(Vector3.{0,1,0}, A.orientation);

    capsule_start_B := B.position;
    capsule_end_B := capsule_start_B + B.capsule_height*rotate(Vector3.{0,1,0}, B.orientation);

    num_contacts, penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B
    := collide_capsule_vs_capsule(capsule_start_A, capsule_end_A, A.radius, capsule_start_B, capsule_end_B, B.radius);

    contacts_count:=num_contacts;
    contacts_data: [4]Contact_Info;

    for 0..contacts_count-1 {
    fill_contact_data_for_wraps(A,B, *contacts_data[it],penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B);
    }

    return contacts_count, contacts_data;
}

wrap_collide_sphere_vs_hull:: (A: *Body, B: *Body) -> contacts_count:int, contacts_data: [4]Contact_Info{
    num_contacts, penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B
    := collide_sphere_vs_hull(A.position, A.radius, B.hull_shape);

    contacts_count:=num_contacts;
    contacts_data: [4]Contact_Info;

    for 0..contacts_count-1 {
        fill_contact_data_for_wraps(A,B, *contacts_data[it],penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B);
    }

    return contacts_count, contacts_data;
}
wrap_collide_capsule_vs_hull:: (A: *Body, B: *Body) -> contacts_count:int, contacts_data: [4]Contact_Info{
        capsule_start_A := A.position;
    capsule_end_A := capsule_start_A + A.capsule_height*rotate(Vector3.{0,1,0}, A.orientation);

    num_contacts, penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B
    := collide_capsule_vs_hull(capsule_start_A, capsule_end_A, A.radius, B.hull_shape);

    contacts_count:=num_contacts;
    contacts_data: [4]Contact_Info;

    for 0..contacts_count-1 {
    fill_contact_data_for_wraps(A,B, *contacts_data[it],penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B);
    }

    return contacts_count, contacts_data;
}
wrap_collide_hull_vs_hull:: (A: *Body, B: *Body) -> contacts_count:int, contacts_data: [4]Contact_Info{

    num_contacts, penetration, world_normal_towards_A, world_midpoint, world_contact_point_on_A, world_contact_point_on_B
    := collide_hull_vs_hull(A.hull_shape ,B.hull_shape);

    contacts_count:=num_contacts;
    contacts_data: [4]Contact_Info;

    for 0..contacts_count-1 {
    fill_contact_data_for_wraps(A,B, *contacts_data[it],penetration, world_normal_towards_A, world_midpoint[it], world_contact_point_on_A[it], world_contact_point_on_B[it]);
    }

    return contacts_count, contacts_data;
}




compute_contacts :: (A: *Body, B: *Body) -> contacts_count:int, contacts_data: [4]Contact_Info
{
    // debug_break();
    basic.assert(A.shape_kind <= B.shape_kind, "SHould ordr them before enering here");
    result_contacts: [4]Contact_Info;
    contacts_count: int;

    if B.shape_kind == .HULL
        draw_hull_properties(B.hull_shape);
    if A.shape_kind |  B.shape_kind == {
           case .SPHERE | .SPHERE;   contacts_count, result_contacts = wrap_collide_sphere_vs_sphere(A, B);
           case .SPHERE | .CAPSULE;  contacts_count, result_contacts = wrap_collide_sphere_vs_capsule(A, B);
           case .SPHERE | .HULL;     contacts_count, result_contacts = wrap_collide_sphere_vs_hull(A, B);
            draw_primitive.circle(A.position, A.radius, normal=rotate(Vector3.{0,1,0}, A.orientation));
            draw_primitive.circle(A.position, A.radius, normal=rotate(Vector3.{1,0,0}, A.orientation));
            draw_primitive.circle(A.position, A.radius, normal=rotate(Vector3.{0,0,1}, A.orientation));
        // case .SPHERE, .MESH;      contacts_count, result_contacts = wrap_collide_sphere_vs_mesh(A, B);
           case .CAPSULE | .CAPSULE; contacts_count, result_contacts = wrap_collide_capsule_vs_capsule(A, B);
           case .CAPSULE | .HULL;    contacts_count, result_contacts = wrap_collide_capsule_vs_hull(A, B);
        // case .CAPSULE, .MESH;     contacts_count, result_contacts = wrap_collide_capsule_vs_mesh(A, B);
           case .HULL | .HULL;       contacts_count, result_contacts = wrap_collide_hull_vs_hull(A, B);
        draw_hull_properties(A.hull_shape);
        // case .HULL | .MESH;    contacts_count, result_contacts = wrap_collide_hull_vs_mesh(A, B);
           case; basic.assert(false, "\n[ERROR]:shape vs shape not implemented! or... remember ordering the shapes in the broadphase!-> %", A.shape_kind |  B.shape_kind); }

    //more sanity checks because entropy code
      for 0..contacts_count -1 {
          basic.assert(is_normalized(result_contacts[it].world_normal_towards_A, tolerance=FLOAT32_EPSILON6), "e world_normal_towards_A %", result_contacts[it].world_normal_towards_A);

        draw_primitive.square(l2w(A.position, A.orientation, result_contacts[it].local_point_on_A),size=0.1, color=.{1,0.1,0.1,1});
        draw_primitive.square(l2w(B.position, B.orientation, result_contacts[it].local_point_on_B),size=0.1, color=.{0.6,0.2,0.6,1});
        if !is_normalized(result_contacts[it].world_normal_towards_A)
        // then continue; //TODO: figure out why the first contact in history is normal 0
        then basic.assert(false); // Debugger Time!

        basic.assert(result_contacts[it].penetration<0); // Debugger Time!
        //these two lines are key to avoid crash at resolve contacts
        //@Important @Check these two lines are key!
        // result_contacts[it].A = A; //:tagAddABtoEachContactPoint
        // result_contacts[it].B = B;
    }


    return contacts_count, result_contacts;
}




//idea, project onto sphere like gauss maps and use angles to.... maybe too slow, angles... sin cos... and I need the axis separation... proejcting onto sphere is information loss.
//wait I think that the idea above might work just needs extra thinking
 Pseudo_Body :: struct {
    bucket_locator: Bucket_Locator; //you can use a pointer or an handle or an id, I use buckets so I use bucketLocator, but I have stable pointers I can use pointers... anyways
    value: float;
    ismin: bool;
}

CompareSAP :: (a: Pseudo_Body, b: Pseudo_Body) -> int {
    if a.value < b.value return -1;
    return 1;
}

Collision_Pair :: struct {
    a,b: Bucket_Locator; //I sohuld use pointers....
}

operator == :: (left: Bucket_Locator, right: Bucket_Locator) -> bool {
    return left.bucket_index==right.bucket_index && left.slot_index == right.slot_index;
}
operator == :: (left: Collision_Pair, right: Collision_Pair) -> bool {
/*
Bucket_Locator :: struct {
    bucket_index: u32;
    slot_index:   s32;  // Signed because we do a trick where we set it to -1.
}
*/
    //it has been 14 hours since I started programming, this code is written after 14hs of work withan extremely tired mind


    return left.a==right.a && left.b == right.b || left.a == right.b && left.b == right.a;
}