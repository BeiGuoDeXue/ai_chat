set(src_dirs 
    inmp441
    max98357
)

set(include_dirs 
    inmp441
    max98357
)

set(requires 
    driver
    esp_common
)

idf_component_register(
    SRC_DIRS ${src_dirs}
    INCLUDE_DIRS ${include_dirs}
    REQUIRES ${requires}
)

component_compile_options(-ffast-math -O3 -Wno-error=format=-Wno-format)