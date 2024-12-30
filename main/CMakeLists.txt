idf_component_register(
    SRCS "main.cpp"
    # SRCS "opus_test.cpp"
    # SRCS "opus_test1.cpp"
    INCLUDE_DIRS "."
    REQUIRES "78__esp-wifi-connect"
             "nvs_flash"
             "78__esp-opus"
             "78__esp-opus-encoder"
             "BSP"
             "78__esp-wifi-connect"
             "json"
)