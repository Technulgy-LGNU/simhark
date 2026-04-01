//! Parallel simulation engine for running many worlds via rayon.

use rayon::prelude::*;

use crate::command::WorldCommand;
use crate::config::WorldConfig;
use crate::domain_randomization::{DomainRandomizer, RandomizationConfig};
use crate::state::WorldState;
use crate::world::World;

/// The main simulation engine managing many parallel worlds.
pub struct SimulationEngine {
    pub worlds: Vec<World>,
}

impl SimulationEngine {
    /// Create `count` worlds, each with the given base config.
    /// Each world gets a unique seed derived from `config.seed + world_index`.
    pub fn new(count: usize, config: WorldConfig) -> Self {
        let worlds: Vec<World> = (0..count)
            .into_par_iter()
            .map(|i| {
                let mut cfg = config.clone();
                cfg.seed = config.seed.wrapping_add(i as u64);
                World::new(i, cfg)
            })
            .collect();
        Self { worlds }
    }

    /// Create worlds with domain randomization applied per-world.
    pub fn new_randomized(
        count: usize,
        config: WorldConfig,
        randomization: RandomizationConfig,
    ) -> Self {
        let randomizer = DomainRandomizer::new(randomization);
        let worlds: Vec<World> = (0..count)
            .into_par_iter()
            .map(|i| {
                let cfg = randomizer.randomize(&config, i);
                World::new(i, cfg)
            })
            .collect();
        Self { worlds }
    }

    /// Number of worlds.
    pub fn count(&self) -> usize {
        self.worlds.len()
    }

    /// Step all worlds with the same command, returning all states.
    pub fn step_all_same(&mut self, command: &WorldCommand) -> Vec<WorldState> {
        self.worlds
            .par_iter_mut()
            .map(|world| world.step(command))
            .collect()
    }

    /// Step all worlds with no commands.
    pub fn step_all(&mut self) -> Vec<WorldState> {
        self.worlds
            .par_iter_mut()
            .map(|world| world.step_empty())
            .collect()
    }

    /// Advance all worlds with the same command without collecting state snapshots.
    pub fn advance_all_same(&mut self, command: &WorldCommand) {
        self.worlds
            .par_iter_mut()
            .for_each(|world| world.advance(command));
    }

    /// Advance all worlds with no commands and without collecting state snapshots.
    pub fn advance_all(&mut self) {
        self.worlds.par_iter_mut().for_each(World::advance_empty);
    }

    /// Advance each world with its own command without collecting state snapshots.
    pub fn advance_with_commands(&mut self, commands: &[WorldCommand]) {
        assert_eq!(
            commands.len(),
            self.worlds.len(),
            "commands length must match world count"
        );
        self.worlds
            .par_iter_mut()
            .zip(commands.par_iter())
            .for_each(|(world, cmd)| world.advance(cmd));
    }

    /// Step each world with its own command. `commands` must have length == count().
    pub fn step_with_commands(&mut self, commands: &[WorldCommand]) -> Vec<WorldState> {
        assert_eq!(
            commands.len(),
            self.worlds.len(),
            "commands length must match world count"
        );
        self.worlds
            .par_iter_mut()
            .zip(commands.par_iter())
            .map(|(world, cmd)| world.step(cmd))
            .collect()
    }

    /// Step a subset of worlds (by index) with individual commands.
    /// Returns states for just those worlds.
    /// Note: indices must not contain duplicates.
    pub fn step_subset(&mut self, indices: &[usize], commands: &[WorldCommand]) -> Vec<WorldState> {
        assert_eq!(indices.len(), commands.len());
        // Sequential for safety (no overlapping mutable borrows)
        indices
            .iter()
            .zip(commands.iter())
            .map(|(&idx, cmd)| self.worlds[idx].step(cmd))
            .collect()
    }

    /// Reset all worlds to initial state.
    pub fn reset_all(&mut self) {
        self.worlds.par_iter_mut().for_each(|world| world.reset());
    }

    /// Reset specific worlds.
    pub fn reset_worlds(&mut self, indices: &[usize]) {
        for &idx in indices {
            self.worlds[idx].reset();
        }
    }

    /// Get current state of all worlds without stepping.
    pub fn get_all_states(&self) -> Vec<WorldState> {
        self.worlds
            .par_iter()
            .map(|world| world.get_state())
            .collect()
    }

    /// Get a mutable reference to a specific world.
    pub fn world_mut(&mut self, index: usize) -> &mut World {
        &mut self.worlds[index]
    }

    /// Get a reference to a specific world.
    pub fn world(&self, index: usize) -> &World {
        &self.worlds[index]
    }
}
