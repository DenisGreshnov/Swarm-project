#include "simulation.h"

FlockSimulation::FlockSimulation() : gamma_target(100, 100), gamma_velocity(0, 0) {
    // Инициализация случайного генератора
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-150, 150);
    
    // Создаем случайных агентов
    for (int i = 0; i < 1000; ++i) {
        agents.emplace_back(Vector2(dis(gen), dis(gen)));
        
        // Добавляем небольшую случайную начальную скорость
        agents.back().velocity = Vector2(dis(gen) * 0.05, dis(gen) * 0.05);
    }
}

// σ-норма из уравнения (8)
double FlockSimulation::sigma_norm(const Vector2& z) const {
    double norm_z = z.length();
    return (1.0 / params.epsilon) * (std::sqrt(1.0 + params.epsilon * norm_z * norm_z) - 1.0);
}

// σ_ε из уравнения (9)
Vector2 FlockSimulation::sigma_epsilon(const Vector2& z) const {
    double norm_z = z.length();
    if (norm_z < 1e-10) return Vector2(0, 0);
    return z * (1.0 / std::sqrt(1.0 + params.epsilon * norm_z * norm_z));
}

// Bump-функция из уравнения (10)
double FlockSimulation::bump_function(double z, double h) const {
    if (z < h) {
        return 1.0;
    } else if (z < 1.0) {
        return 0.5 * (1.0 + std::cos(M_PI * (z - h) / (1.0 - h)));
    } else {
        return 0.0;
    }
}

// Матрица смежности для α-агентов
double FlockSimulation::alpha_adjacency(const Vector2& q_i, const Vector2& q_j) const {
    double distance = sigma_norm(q_j - q_i);
    double r_alpha = sigma_norm(Vector2(params.interaction_range, 0));
    return bump_function(distance / r_alpha, params.h_alpha);
}

// Матрица смежности для β-агентов  
double FlockSimulation::beta_adjacency(const Vector2& q_i, const Vector2& obstacle_pos) const {
    double distance = sigma_norm(obstacle_pos - q_i);
    double d_beta = sigma_norm(Vector2(params.desired_distance * 0.6, 0)); // d_β
    return bump_function(distance / d_beta, params.h_beta); // использовать d_β
}

// Функция действия φ_α из уравнения (15)
double FlockSimulation::phi_alpha(double z) const {
    double d_alpha = sigma_norm(Vector2(params.desired_distance, 0));
    double r_alpha = sigma_norm(Vector2(params.interaction_range, 0));
    
    // Упрощенная версия - можно расширить согласно уравнению (15)
    double bump = bump_function(z / r_alpha, params.h_alpha);
    double action = (z - d_alpha) / std::sqrt(1.0 + (z - d_alpha) * (z - d_alpha));
    
    return bump * action;
}

// Функция действия φ_β из уравнения (65)
double FlockSimulation::phi_beta(double z) const {
    double d_beta = sigma_norm(Vector2(params.desired_distance * 0.6, 0));
    
    double bump = bump_function(z / d_beta, params.h_beta); // z/d_β, а не z/r_β
    
    // Правильная реализация по уравнению (65)
    double s = z - d_beta;
    double sigma1 = s / std::sqrt(1.0 + s * s); // σ_1(z - d_β)
    double action = sigma1 - 1.0;
    
    return bump * action;
}

void FlockSimulation::step(double delta_time) {
    std::lock_guard<std::mutex> lock(data_mutex);
    
    // Обновляем β-агентов
    update_beta_agents();
    
    // Обновляем ускорения для всех агентов согласно Algorithm 3
    for (auto& agent : agents) {
        Vector2 alpha_force = compute_alpha_force(agent);
        Vector2 beta_force = compute_beta_force(agent);
        Vector2 gamma_force = compute_gamma_force(agent);
        
        // Суммируем все силы согласно уравнению (67)
        agent.acceleration = alpha_force + beta_force + gamma_force;
    }
    
    // Обновляем позиции и скорости
    for (auto& agent : agents) {
        // Интегрирование скорости (уравнение движения (2))
        agent.velocity = agent.velocity + agent.acceleration * delta_time;
        
        // Ограничение максимальной скорости для стабильности
        double speed = agent.velocity.length();
        double max_speed = 100.0;
        if (speed > max_speed) {
            agent.velocity = agent.velocity.normalized() * max_speed;
        }
        
        // Интегрирование позиции
        agent.position = agent.position + agent.velocity * delta_time;
        
        // Мягкое ограничение области
        const double boundary = 200.0;
        const double soft_boundary = 180.0;
        
        if (std::abs(agent.position.x) > soft_boundary) {
            double push = (boundary - std::abs(agent.position.x)) / (boundary - soft_boundary);
            agent.velocity.x += (agent.position.x > 0 ? -1 : 1) * push * 5.0;
        }
        if (std::abs(agent.position.y) > soft_boundary) {
            double push = (boundary - std::abs(agent.position.y)) / (boundary - soft_boundary);
            agent.velocity.y += (agent.position.y > 0 ? -1 : 1) * push * 5.0;
        }
    }
}

Vector2 FlockSimulation::compute_alpha_force(const Agent& agent) {
    Vector2 gradient_force(0, 0);
    Vector2 consensus_force(0, 0);
    
    for (const auto& other : agents) {
        if (&agent == &other) continue;
        
        Vector2 diff = other.position - agent.position;
        double distance = diff.length();
        
        if (distance < params.interaction_range && distance > 0.1) {
            // Градиентный член из уравнения (68)
            double z = sigma_norm(diff);
            Vector2 n_ij = sigma_epsilon(diff);
            gradient_force = gradient_force + n_ij * phi_alpha(z);
            
            // Консенсусный член (velocity matching) из уравнения (68)
            double a_ij = alpha_adjacency(agent.position, other.position);
            consensus_force = consensus_force + (other.velocity - agent.velocity) * a_ij;
        }
    }
    
    return gradient_force * params.c1_alpha + consensus_force * params.c2_alpha;
}

Vector2 FlockSimulation::compute_beta_force(const Agent& agent) {
    Vector2 repulsion_force(0, 0);
    Vector2 damping_force(0, 0);
    
    for (const auto& beta_agent : beta_agents) {
        Vector2 diff = beta_agent.position - agent.position;
        double distance = diff.length();
        
        if (distance < params.obstacle_range && distance > 0.1) {
            // Отталкивающий член из уравнения (69)
            double z = sigma_norm(diff);
            Vector2 n_ik = sigma_epsilon(diff);
            repulsion_force = repulsion_force + n_ik * phi_beta(z);
            
            // Демпфирующий член из уравнения (69)
            double b_ik = beta_adjacency(agent.position, beta_agent.position);
            damping_force = damping_force + (beta_agent.velocity - agent.velocity) * b_ik;
        }
    }
    
    return repulsion_force * params.c1_beta + damping_force * params.c2_beta;
}

Vector2 FlockSimulation::compute_gamma_force(const Agent& agent) {
    if (!use_gamma_target) return Vector2(0, 0);
    
    Vector2 diff = agent.position - gamma_target;
    double norm_diff = diff.length();
    
    // Правильная σ_1 по уравнению (70)
    Vector2 position_term = (norm_diff < 1e-10) ? 
        Vector2(0,0) : diff * (1.0 / std::sqrt(1.0 + norm_diff * norm_diff));
    
    Vector2 velocity_term = agent.velocity - gamma_velocity;
    
    return position_term * (-params.c1_gamma) - velocity_term * params.c2_gamma;
}

void FlockSimulation::update_beta_agents() {
    beta_agents.clear();
    
    // Для каждого агента проверяем близкие препятствия и создаем β-агентов
    for (const auto& agent : agents) {
        for (const auto& obstacle : obstacles) {
            Vector2 to_obstacle = obstacle.position - agent.position;
            double distance = to_obstacle.length();
            
            if (distance < params.obstacle_range + obstacle.radius) {
                BetaAgent beta_agent = project_to_obstacle(agent, obstacle);
                beta_agents.push_back(beta_agent);
            }
        }
    }
}

BetaAgent FlockSimulation::project_to_obstacle(const Agent& agent, const Obstacle& obstacle) const {
    BetaAgent beta_agent;
    
    if (obstacle.is_wall) {
        // Проекция на стену (гиперплоскость)
        // Упрощенная версия - предполагаем горизонтальную/вертикальную стену
        if (std::abs(obstacle.position.x - agent.position.x) < std::abs(obstacle.position.y - agent.position.y)) {
            // Горизонтальная стена
            beta_agent.position = Vector2(agent.position.x, obstacle.position.y);
            beta_agent.velocity = Vector2(agent.velocity.x, 0); // скорость параллельна стене
        } else {
            // Вертикальная стена  
            beta_agent.position = Vector2(obstacle.position.x, agent.position.y);
            beta_agent.velocity = Vector2(0, agent.velocity.y);
        }
    } else {
        // Проекция на сферическое препятствие
        Vector2 to_center = obstacle.position - agent.position;
        double distance_to_center = to_center.length();
        double mu = obstacle.radius / distance_to_center;
        if (distance_to_center > 0.1) {
            Vector2 direction = to_center.normalized();
            beta_agent.position = obstacle.position - direction * obstacle.radius;
            // Проекция скорости на касательную плоскость
            beta_agent.velocity = (agent.velocity - direction * agent.velocity.dot(direction)) * mu;
        } else {
            beta_agent.position = obstacle.position + Vector2(obstacle.radius, 0);
            beta_agent.velocity = Vector2(0, 0);
        }
    }
    
    return beta_agent;
}

void FlockSimulation::add_obstacle(const Vector2& position, double radius) {
    std::lock_guard<std::mutex> lock(data_mutex);
    obstacles.emplace_back(position, radius, false); // сферическое препятствие
    std::cout << "Added obstacle at (" << position.x << ", " << position.y 
              << ") with radius " << radius << std::endl;
}

void FlockSimulation::set_target(const Vector2& target) {
    std::lock_guard<std::mutex> lock(data_mutex);
    gamma_target = target;
    use_gamma_target = true; // Автоматически включаем цель при установке
    std::cout << "Target set to (" << target.x << ", " << target.y << ")" << std::endl;
}

void FlockSimulation::clear_obstacles() {
    std::lock_guard<std::mutex> lock(data_mutex);
    obstacles.clear();
    beta_agents.clear();
    std::cout << "All obstacles cleared" << std::endl;
}

std::vector<Agent> FlockSimulation::get_agents() const{
    std::lock_guard<std::mutex> lock(data_mutex);
    return agents;
}

std::vector<Obstacle> FlockSimulation::get_obstacles() const{
    std::lock_guard<std::mutex> lock(data_mutex);
    return obstacles;
}

std::vector<BetaAgent> FlockSimulation::get_beta_agents() const{
    std::lock_guard<std::mutex> lock(data_mutex);
    return beta_agents;
}

Vector2 FlockSimulation::get_target() const{
    std::lock_guard<std::mutex> lock(data_mutex);
    return gamma_target;
}