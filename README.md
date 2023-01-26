# Parallel Search-Based Planning Algorithms

This repository maintains open-source implementations of several parallel search-based planning algorithms. Currently, the following algorithms are available:
* [wA*: Weighted A*](https://www.sciencedirect.com/science/article/pii/000437027090007X) [Pohl 1970]
* PwA*: Parallel Weighted A* (As explained in the [ePA\*SE](https://arxiv.org/pdf/2203.01369.pdf) and [MPLP](https://ieeexplore.ieee.org/abstract/document/9730025) papers)
* [PA\*SE: Parallel A* For Slow Expansions](https://www.aaai.org/ocs/index.php/ICAPS/ICAPS14/paper/view/7952/8029) [Phillips et al. 2014]
* [MPLP: Massively Parallelized Lazy Planning](https://arxiv.org/abs/2107.02826) [Mukherjee et al. 2022]
* [ePA\*SE: Edge-based Parallel A* For Slow Evaluations](https://arxiv.org/abs/2203.01369) [Mukherjee et al. 2022]
* [GePA\*SE: Generalized Edge-Based Parallel A* for Slow Evaluations](https://arxiv.org/abs/2301.10347) [Mukherjee et al. 2023]

If you use MPLP, please use the following citation:
```
@article{mukherjee2022mplp,
	title        = {{MPLP}: Massively Parallelized Lazy Planning},
	author       = {Mukherjee, Shohin and Aine, Sandip and Likhachev, Maxim},
	year         = 2022,
	journal      = {IEEE Robotics and Automation Letters},
	publisher    = {IEEE},
	volume       = 7,
	number       = 3,
	pages        = {6067--6074}
}
```
If you use ePA*SE, please use the following citation:
```
@inproceedings{mukherjee2022epase,
  author    = {Mukherjee, Shohin and Aine, Sandip and Likhachev, Maxim},
  title     = {{ePA*SE}: Edge-Based Parallel {A*} for Slow Evaluations},
  booktitle = {International Symposium on Combinatorial Search},
  volume={15},
  number={1},
  pages={136--144},
  publisher={{AAAI} Press},
  year={2022}
}
```
If you use GePA*SE, please use the following citation:
```
@article{mukherjee2023gepase,
  title = {{GePA*SE}: Generalized Edge-Based Parallel A* for Slow Evaluations},
  author = {Mukherjee, Shohin and Likhachev, Maxim},
  year = {2023},
  journal = {arXiv preprint arXiv:2301.10347},
}
```

## Usage

### Dependencies
The planning code only uses C++ standard libraries and has no external dependencies. However, the robot_nav_2d example has the following dependencies:
1. [Boost C++](https://www.boost.org/)
2. OpenCV C++ for visualization
3. [sbpl package](https://github.com/sbpl/sbpl)

### Building
1. Clone the repository.
    ```
    git clone https://github.com/shohinm/parallel_search.git
    ```
2. Create build directory and compile.
    ```
    cd parallel_search && mkdir build && cd build && cmake ../ -DCMAKE_BUILD_TYPE=Release && make
    ```
    
### Running the example
Run the following commands from within the build directory:
* wA*
    ```
    ./run_robot_nav_2d wastar
    ```
* PwA* with 5 threads
    ```
    ./run_robot_nav_2d pwastar 5
    ```
* MPLP with 5 threads
    ```
    ./run_robot_nav_2d mplp 5
    ```  
* PA*SE with 5 threads
    ```
    ./run_robot_nav_2d pase 5
    ```
* ePA*SE with 5 threads
    ```
    ./run_robot_nav_2d epase 5
    ```
* GePA*SE with 5 threads
    ```
    ./run_robot_nav_2d gepase 5
    ```
    
**Maintained by** : [Shohin Mukherjee](https://www.ri.cmu.edu/ri-people/shohin-mukherjee/)
