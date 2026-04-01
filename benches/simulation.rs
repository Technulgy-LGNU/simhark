//! Benchmarks for parallel simulation throughput.

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use simhark::command::*;
use simhark::domain_randomization::*;
use simhark::*;

fn bench_world_creation(c: &mut Criterion) {
    let config = WorldConfig::division_a();
    c.bench_function("create_single_world", |b| {
        b.iter(|| World::new(0, config.clone()))
    });
}

fn bench_single_step(c: &mut Criterion) {
    let config = WorldConfig::division_a();
    let mut world = World::new(0, config);
    let cmd = WorldCommand::default();

    c.bench_function("single_world_step", |b| b.iter(|| world.step(&cmd)));
}

fn bench_parallel_steps(c: &mut Criterion) {
    let mut group = c.benchmark_group("parallel_step");

    for &count in &[16, 64, 256, 1024] {
        let config = WorldConfig::division_a();
        let mut engine = SimulationEngine::new(count, config);

        group.bench_with_input(BenchmarkId::new("worlds", count), &count, |b, _| {
            b.iter(|| engine.step_all())
        });
    }
    group.finish();
}

fn bench_parallel_advance(c: &mut Criterion) {
    let mut group = c.benchmark_group("parallel_advance");

    for &count in &[16, 64, 256, 1024] {
        let config = WorldConfig::division_a();
        let mut engine = SimulationEngine::new(count, config);

        group.bench_with_input(BenchmarkId::new("worlds", count), &count, |b, _| {
            b.iter(|| engine.advance_all())
        });
    }
    group.finish();
}

fn bench_parallel_with_commands(c: &mut Criterion) {
    let mut group = c.benchmark_group("parallel_step_with_commands");

    for &count in &[64, 256] {
        let config = WorldConfig::division_a();
        let mut engine = SimulationEngine::new(count, config);

        let commands: Vec<WorldCommand> = (0..count)
            .map(|_| WorldCommand {
                blue: (0..11)
                    .map(|id| RobotCommand {
                        id,
                        move_command: Some(MoveCommand::LocalVelocity {
                            forward: 1.0,
                            left: 0.0,
                            angular: 0.5,
                        }),
                        kick_speed: 0.0,
                        kick_angle: 0.0,
                        dribbler_on: false,
                    })
                    .collect(),
                ..Default::default()
            })
            .collect();

        group.bench_with_input(BenchmarkId::new("worlds", count), &count, |b, _| {
            b.iter(|| engine.step_with_commands(&commands))
        });
    }
    group.finish();
}

fn bench_randomized_creation(c: &mut Criterion) {
    let config = WorldConfig::division_a();
    c.bench_function("create_256_randomized_worlds", |b| {
        b.iter(|| {
            SimulationEngine::new_randomized(256, config.clone(), RandomizationConfig::moderate())
        })
    });
}

criterion_group!(
    benches,
    bench_world_creation,
    bench_single_step,
    bench_parallel_steps,
    bench_parallel_advance,
    bench_parallel_with_commands,
    bench_randomized_creation,
);
criterion_main!(benches);
