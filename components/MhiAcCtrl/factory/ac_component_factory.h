/**
 * @file ac_component_factory.h
 * @brief Factory pattern implementation for creating AC control components
 * @author ESPHome MHI AC Control Team
 * @version 4.3
 * @date 2025-01-15
 * 
 * This file implements the Factory pattern for creating different types of
 * AC control components in a type-safe and extensible manner.
 */

#pragma once

#include "../enums/ac_enums.h"
#include "../interfaces/ac_component_interface.h"
#include "../interfaces/ac_configuration_interface.h"
#include <memory>
#include <unordered_map>
#include <functional>

namespace esphome {
namespace mhi {
namespace factory {

/**
 * @class ACComponentFactory
 * @brief Factory class for creating AC control components
 * 
 * This factory follows the Factory Method pattern and provides a centralized
 * way to create different types of AC control components. It supports both
 * compile-time and runtime component creation.
 */
class ACComponentFactory {
public:
    /**
     * @brief Constructor
     * @param config_manager Shared pointer to configuration manager
     */
    explicit ACComponentFactory(std::shared_ptr<interfaces::ACConfigurationInterface> config_manager);
    
    /**
     * @brief Destructor
     */
    virtual ~ACComponentFactory() = default;

    /**
     * @brief Create a component of the specified type
     * @tparam T Component type
     * @param component_type Type of component to create
     * @param config Configuration for the component
     * @return Shared pointer to created component
     */
    template<typename T>
    std::shared_ptr<T> createComponent(enums::ACComponentType component_type, 
                                     const interfaces::ACComponentConfig& config);

    /**
     * @brief Register a component creator function
     * @param component_type Type of component
     * @param creator Function that creates the component
     */
    void registerComponentCreator(enums::ACComponentType component_type,
                                std::function<std::shared_ptr<interfaces::ACComponentInterface>()> creator);

    /**
     * @brief Create a climate control component
     * @param config Configuration for the component
     * @return Shared pointer to climate component
     */
    std::shared_ptr<interfaces::ACComponentInterface> createClimateComponent(
        const interfaces::ACComponentConfig& config);

    /**
     * @brief Create a sensor component
     * @param config Configuration for the component
     * @return Shared pointer to sensor component
     */
    std::shared_ptr<interfaces::ACComponentInterface> createSensorComponent(
        const interfaces::ACComponentConfig& config);

    /**
     * @brief Create a select component
     * @param config Configuration for the component
     * @return Shared pointer to select component
     */
    std::shared_ptr<interfaces::ACComponentInterface> createSelectComponent(
        const interfaces::ACComponentConfig& config);

    /**
     * @brief Create a switch component
     * @param config Configuration for the component
     * @return Shared pointer to switch component
     */
    std::shared_ptr<interfaces::ACComponentInterface> createSwitchComponent(
        const interfaces::ACComponentConfig& config);

    /**
     * @brief Create a binary sensor component
     * @param config Configuration for the component
     * @return Shared pointer to binary sensor component
     */
    std::shared_ptr<interfaces::ACComponentInterface> createBinarySensorComponent(
        const interfaces::ACComponentConfig& config);

    /**
     * @brief Create a text sensor component
     * @param config Configuration for the component
     * @return Shared pointer to text sensor component
     */
    std::shared_ptr<interfaces::ACComponentInterface> createTextSensorComponent(
        const interfaces::ACComponentConfig& config);

    /**
     * @brief Get the configuration manager
     * @return Shared pointer to configuration manager
     */
    std::shared_ptr<interfaces::ACConfigurationInterface> getConfigurationManager() const;

    /**
     * @brief Set the configuration manager
     * @param config_manager Shared pointer to configuration manager
     */
    void setConfigurationManager(std::shared_ptr<interfaces::ACConfigurationInterface> config_manager);

private:
    std::shared_ptr<interfaces::ACConfigurationInterface> config_manager_;
    std::unordered_map<enums::ACComponentType, 
                      std::function<std::shared_ptr<interfaces::ACComponentInterface>()>> creators_;

    /**
     * @brief Initialize default component creators
     */
    void initializeDefaultCreators();

    /**
     * @brief Validate component configuration
     * @param config Configuration to validate
     * @return Validation result
     */
    enums::ACValidationResult validateConfiguration(const interfaces::ACComponentConfig& config);
};

/**
 * @class ACComponentBuilder
 * @brief Builder pattern implementation for complex component creation
 * 
 * This builder provides a fluent interface for creating complex AC components
 * with multiple configuration options.
 */
class ACComponentBuilder {
public:
    /**
     * @brief Constructor
     * @param factory Reference to component factory
     */
    explicit ACComponentBuilder(ACComponentFactory& factory);

    /**
     * @brief Set component type
     * @param type Component type
     * @return Reference to builder for method chaining
     */
    ACComponentBuilder& setType(enums::ACComponentType type);

    /**
     * @brief Set component name
     * @param name Component name
     * @return Reference to builder for method chaining
     */
    ACComponentBuilder& setName(const std::string& name);

    /**
     * @brief Set component configuration
     * @param config Configuration object
     * @return Reference to builder for method chaining
     */
    ACComponentBuilder& setConfig(const interfaces::ACComponentConfig& config);

    /**
     * @brief Add a property to the component
     * @param key Property key
     * @param value Property value
     * @return Reference to builder for method chaining
     */
    ACComponentBuilder& addProperty(const std::string& key, const std::string& value);

    /**
     * @brief Build the component
     * @return Shared pointer to created component
     */
    std::shared_ptr<interfaces::ACComponentInterface> build();

private:
    ACComponentFactory& factory_;
    enums::ACComponentType type_;
    std::string name_;
    interfaces::ACComponentConfig config_;
    std::unordered_map<std::string, std::string> properties_;
};

} // namespace factory
} // namespace mhi
} // namespace esphome
