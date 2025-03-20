# Subset-Based Collective Decision-Making: Maintaining Decision Accuracy with Fewer Robots

*Samratul Fuady, Danesh Tarapore, and Mohammad D. Soorati*

This repository contains the code accompanying our paper, which has been deployed on the ARGoS simulator.

## Running the Code  

### Prerequisites  
Before running the simulations, you need to install [ARGoS](https://www.argos-sim.info/index.php).  

### Compilation  
To compile the project, follow these steps:

1. **Create and enter the build directory**  
   First, create a `build` directory (if it doesn't already exist) and navigate into it:  
   ```sh
   mkdir -p build && cd build
    ```
2. **Configure the build system**  
   Use `cmake` to configure the project and generate the necessary build files:  
   ```sh
   cmake ..
    ```
3. **Compile the project**  
   Run `make` to compile the project using the generated build files:  
   ```sh
   make
    ```
4. **Return to the root directory**  
   After the build is complete, return to the project's root directory.

### Running a Simulation  
To run a simulation, execute the appropriate configuration file:  

- **Leader-based strategy:**  
  ```sh
  argos3 -c experiments/leader_subset.argos
  ```

- **Distributed strategy:**  
  ```sh
  argos3 -c experiments/dist_subset.argos
  ```

  