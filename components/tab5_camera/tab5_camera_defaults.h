/*  tab5_camera_defaults.h – valeurs par défaut utilisées quand le projet
 *  n’est pas configuré via menuconfig.
 *  Vous pouvez remplacer ce fichier par un vrai menuconfig si vous le désirez.
 */
#pragma once

/* Limite de gain absolu (gain ×1000) – valeur typique pour le SC202CS */
#define CONFIG_CAMERA_SC202CS_ABSOLUTE_GAIN_LIMIT   64000

/* Nombre maximal de formats supportés (défini dans le tableau sc202cs_format_info[]) */
#define CONFIG_CAMERA_SC202CS_MAX_SUPPORT           4

/* Mode de priorité du gain : 1 = analog‑gain‑priority, 0 = digital‑gain‑priority */
#define CONFIG_CAMERA_SC202CS_ANA_GAIN_PRIORITY    1

/* Index du format par défaut dans sc202cs_format_info[] */
#define CONFIG_CAMERA_SC202CS_MIPI_IF_FORMAT_INDEX_DEFAULT 0
