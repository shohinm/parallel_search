
# Parallel Search-Based Planning Algorithms

This repository maintains open-source implementations of several parallel search-based planning algorithms. Currently, the following algorithms are available:
* [wA*: Weighted A*](https://www.sciencedirect.com/science/article/pii/000437027090007X) [Pohl 1970]
* [PA\*SE: Parallel A* For Slow Evaluations](https://www.aaai.org/ocs/index.php/ICAPS/ICAPS14/paper/view/7952/8029) [Phillips et al. 2014]
* [ePA\*SE: Edge-based Parallel A* For Slow Evaluations](https://arxiv.org/pdf/2203.01369.pdf) [Mukherjee et al. 2022]

If you use ePA*SE, please use the following citation:
```
@article{mukherjee2022epa,
  title={ePA* SE: Edge-based Parallel A* for Slow Evaluations},
  author={Mukherjee, Shohin and Aine, Sandip and Likhachev, Maxim},
  journal={arXiv preprint arXiv:2203.01369},
  year={2022}
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
    ./robot_nav_2d wastar
    ```
* PA*SE with 5 threads
    ```
    ./robot_nav_2d wastar pase 5
    ```
* ePA*SE with 5 threads
    ```
    ./robot_nav_2d wastar epase 5
    ```
  
    
**Maintained by** : [Shohin Mukherjee](https://www.ri.cmu.edu/ri-people/shohin-mukherjee/)
