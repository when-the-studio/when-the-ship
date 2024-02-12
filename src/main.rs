use ggez::event;
use ggez::glam::*;
use ggez::graphics::{self, Color};
use ggez::{Context, GameResult};

use rapier2d::prelude::*;

const SCALING_FACTOR: f32 = 10.0;

fn world_to_screen(world_coords: Vector<Real>) -> Vec2 {
	Vec2::new(
		world_coords.x * SCALING_FACTOR,
		800.0 - world_coords.y * SCALING_FACTOR,
	)
}

fn world_length_to_screen(world_length: f32) -> f32 {
	world_length * SCALING_FACTOR
}

fn screen_to_world(screen_coords: Vec2) -> Vector<Real> {
	vector![
		screen_coords.x / SCALING_FACTOR,
		(800.0 - screen_coords.y) / SCALING_FACTOR,
	]
}
fn _screen_length_to_world(screen_length: f32) -> f32 {
	screen_length / SCALING_FACTOR
}

struct Ball {
	rigid_body_handle: RigidBodyHandle,
	world_radius: f32,
}

struct PhysicalObjectSets {
	bodies: RigidBodySet,
	colliders: ColliderSet,
	impulse_joints: ImpulseJointSet,
	multibody_joints: MultibodyJointSet,
}

struct Physics {
	gravity: Vector<Real>,
	integration_parameters: IntegrationParameters,
	physics_pipeline: PhysicsPipeline,
	island_manager: IslandManager,
	broad_phase: BroadPhase,
	narrow_phase: NarrowPhase,
	ccd_solver: CCDSolver,
	query_pipeline: QueryPipeline,
	physics_hooks: (),
	event_handler: (),
}

struct MainState {
	ball: Ball,
	phys_objs: PhysicalObjectSets,
	physics: Physics,
}

impl MainState {
	fn new() -> GameResult<MainState> {
		let mut bodies = RigidBodySet::new();
		let mut colliders = ColliderSet::new();
		let impulse_joints = ImpulseJointSet::new();
		let multibody_joints = MultibodyJointSet::new();

		let radius = 0.5;
		let rigid_body = RigidBodyBuilder::dynamic()
			.translation(screen_to_world(Vec2::new(400.0, 400.0)))
			.linear_damping(0.6)
			.build();
		let collider = ColliderBuilder::ball(radius).restitution(0.4).build();
		let rigid_body_handle = bodies.insert(rigid_body);
		colliders.insert_with_parent(collider, rigid_body_handle, &mut bodies);

		let s = MainState {
			ball: Ball {
				rigid_body_handle,
				world_radius: 2.0,
			},
			phys_objs: PhysicalObjectSets {
				bodies,
				colliders,
				impulse_joints,
				multibody_joints,
			},
			physics: Physics {
				gravity: vector![0.0, -10.0],
				integration_parameters: IntegrationParameters::default(),
				physics_pipeline: PhysicsPipeline::new(),
				island_manager: IslandManager::new(),
				broad_phase: BroadPhase::new(),
				narrow_phase: NarrowPhase::new(),
				ccd_solver: CCDSolver::new(),
				query_pipeline: QueryPipeline::new(),
				physics_hooks: (),
				event_handler: (),
			},
		};
		Ok(s)
	}
}

impl event::EventHandler<ggez::GameError> for MainState {
	fn update(&mut self, ctx: &mut Context) -> GameResult {
		let dt = ctx.time.delta().as_secs_f32();
		self.physics.integration_parameters.dt = dt;

		self.physics.physics_pipeline.step(
			&self.physics.gravity,
			&self.physics.integration_parameters,
			&mut self.physics.island_manager,
			&mut self.physics.broad_phase,
			&mut self.physics.narrow_phase,
			&mut self.phys_objs.bodies,
			&mut self.phys_objs.colliders,
			&mut self.phys_objs.impulse_joints,
			&mut self.phys_objs.multibody_joints,
			&mut self.physics.ccd_solver,
			Some(&mut self.physics.query_pipeline),
			&self.physics.physics_hooks,
			&self.physics.event_handler,
		);
		Ok(())
	}

	fn draw(&mut self, ctx: &mut Context) -> GameResult {
		let mut canvas =
			graphics::Canvas::from_frame(ctx, graphics::Color::from([0.1, 0.2, 0.3, 1.0]));
		let position =
			world_to_screen(*self.phys_objs.bodies[self.ball.rigid_body_handle].translation());
		let radius = world_length_to_screen(self.ball.world_radius);

		let circle = graphics::Mesh::new_circle(
			ctx,
			graphics::DrawMode::fill(),
			Vec2::new(0.0, 0.0),
			radius,
			0.01,
			Color::WHITE,
		)?;
		canvas.draw(&circle, position);

		canvas.finish(ctx)?;
		Ok(())
	}
}

pub fn main() -> GameResult {
	let cb = ggez::ContextBuilder::new("super_simple", "ggez");
	let (ctx, event_loop) = cb.build()?;
	let state = MainState::new()?;
	event::run(ctx, event_loop, state)
}
