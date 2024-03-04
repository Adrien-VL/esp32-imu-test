set(FUSION_SRCS
    "fusion.c"
    "include/FusionAhrs.c"
    "include/FusionCompass.c"
    "include/FusionOffset.c")

idf_component_register(SRCS ${FUSION_SRCS}
                    INCLUDE_DIRS "include" ".")
