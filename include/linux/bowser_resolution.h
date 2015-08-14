#ifndef _BOWSER_RESOLUTION_H_
#define _BOWSER_RESOLUTION_H_

enum bowser_resolution_state {
	BOWSER_RESOLUTION_HD = 0,
	BOWSER_RESOLUTION_FHD,
};

/**
 * Defines different vendors of displays.
 */
typedef enum
{
    NvOdmBoardDisplayPanelId_Default = 0,
    NvOdmBoardDisplayPanelId_Sharp,
    NvOdmBoardDisplayPanelId_LG,
    NvOdmBoardDisplayPanelId_AUO_14_FHD,
    NvOdmBoardDisplayPanelId_AUO_14_HD,
} NvOdmBoardDisplayPanelId;

extern bool panel_initialized;

#endif // End of #ifndef _BOWSER_RESOLUTION_H_
