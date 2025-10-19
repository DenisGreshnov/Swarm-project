#pragma once
#include <vector>
#include <cmath>
#include <atomic>
#include <mutex>
#include <iostream>
#include <random>

// Простой класс вектора для 2D
struct Vector2 {
    double x, y;
    
    Vector2(double x = 0, double y = 0) : x(x), y(y) {}
    
    Vector2 operator+(const Vector2& other) const {
        return Vector2(x + other.x, y + other.y);
    }
    
    Vector2 operator-(const Vector2& other) const {
        return Vector2(x - other.x, y - other.y);
    }
    
    Vector2 operator*(double scalar) const {
        return Vector2(x * scalar, y * scalar);
    }
    
    double dot(const Vector2& other) const {
        return x * other.x + y * other.y;
    }
    
    double length() const {
        return std::sqrt(x*x + y*y);
    }
    
    Vector2 normalized() const {
        double len = length();
        if (len < 1e-10) return Vector2(0, 0);
        return Vector2(x/len, y/len);
    }
};

// α-агент
struct Agent {
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration;
    
    Agent(Vector2 pos = Vector2(0, 0)) : position(pos), velocity(0, 0), acceleration(0, 0) {}
};

// β-агент (препятствие)
struct BetaAgent {
    Vector2 position;
    Vector2 velocity;
    
    BetaAgent(Vector2 pos = Vector2(0, 0)) : position(pos), velocity(0, 0) {}
};

// Препятствие
struct Obstacle {
    Vector2 position;
    double radius;
    bool is_wall;
    Vector2 wall_normal; // для стен
    
    Obstacle(Vector2 pos = Vector2(0, 0), double r = 15.0, bool wall = false) 
        : position(pos), radius(r), is_wall(wall) {}
};

// Основной класс симуляции
class FlockSimulation {
private:
    std::vector<Agent> agents;
    std::vector<Obstacle> obstacles;
    std::vector<BetaAgent> beta_agents; // β-агенты для препятствий
    Vector2 gamma_target;
    Vector2 gamma_velocity;
    
    mutable std::mutex data_mutex; // mutable для const методов
    std::atomic<bool> running{false};
    
    // Флаги управления
    bool show_beta_agents = false;
    bool use_gamma_target = true;
    bool show_connections = false; // НОВОЕ: отображение сетки связей
    
    // Параметры Algorithm 3 из статьи
    struct Parameters {
        // Основные параметры
        double desired_distance = 7; // d
        double interaction_range = 8.4; // r = 1.2 * d
        double obstacle_range = 5.2; // r' = 0.6 * r
        
        // Коэффициенты сил
        double c1_alpha = 8.0; // для α-взаимодействий
        double c2_alpha = 6.0; // демпфирование α
        double c1_beta = 5.0;  // для β-взаимодействий  
        double c2_beta = 2.0;  // демпфирование β
        double c1_gamma = 0.5; // для навигации
        double c2_gamma = 0.8; // демпфирование навигации
        
        // Параметры σ-нормы
        double epsilon = 0.1;
        
        // Параметры bump-функций
        double h_alpha = 0.2;
        double h_beta = 0.8;
        
    } params;

public:
    FlockSimulation();
    
    void step(double delta_time);
    void add_obstacle(const Vector2& position, double radius = 15.0);
    void set_target(const Vector2& target);
    void clear_obstacles();
    
    // Методы для получения данных для рендеринга - теперь const
    std::vector<Agent> get_agents() const;
    std::vector<Obstacle> get_obstacles() const;
    std::vector<BetaAgent> get_beta_agents() const;
    Vector2 get_target() const;
    
    // Геттеры параметров для рендеринга
    double get_interaction_range() const { return params.interaction_range; }
    double get_obstacle_range() const { return params.obstacle_range; }
    
    // Новые методы управления - теперь const где необходимо
    void toggle_beta_display() { show_beta_agents = !show_beta_agents; }
    void remove_target() { use_gamma_target = false; }
    void enable_target() { use_gamma_target = true; }
    void toggle_connections() { show_connections = !show_connections; } // НОВОЕ
    
    bool is_target_enabled() const { return use_gamma_target; }
    bool is_beta_display_enabled() const { return show_beta_agents; }
    bool is_connections_display_enabled() const { return show_connections; } // НОВОЕ
    
    bool is_running() const { return running; }
    void start() { running = true; }
    void stop() { running = false; }

private:
    // Вспомогательные математические функции
    double sigma_norm(const Vector2& z) const;
    Vector2 sigma_epsilon(const Vector2& z) const;
    double bump_function(double z, double h) const;
    
    // Вычисление матриц смежности
    double alpha_adjacency(const Vector2& q_i, const Vector2& q_j) const;
    double beta_adjacency(const Vector2& q_i, const Vector2& obstacle_pos) const;
    
    // Функции действия (action functions)
    double phi_alpha(double z) const;
    double phi_beta(double z) const;
    
    // Внутренние методы вычисления сил согласно Algorithm 3
    Vector2 compute_alpha_force(const Agent& agent);
    Vector2 compute_beta_force(const Agent& agent);
    Vector2 compute_gamma_force(const Agent& agent);
    
    // Обновление β-агентов
    void update_beta_agents();
    
    // Проекция на препятствия для создания β-агентов
    BetaAgent project_to_obstacle(const Agent& agent, const Obstacle& obstacle) const;
};