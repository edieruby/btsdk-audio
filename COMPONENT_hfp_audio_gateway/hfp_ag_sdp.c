/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 * This file contains SDP functionality required HF Device sample application.
 * SDP database definition is contained in this file and is not changed from
 * the controlling MCU.
 *
 */
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_cfg.h"
#include "hci_control_api.h"
#include "hfp_ag.h"
#include "wiced_memory.h"

extern void hfp_ag_process_open_callback( hfp_ag_session_cb_t *p_scb, uint8_t status );
extern void hfp_ag_rfcomm_do_open( hfp_ag_session_cb_t *p_scb );
extern wiced_timer_t sdp_timer;               /* wiced bt app sdp timer */

/******************************************************
*                 Global Variables
******************************************************/
static wiced_bt_uuid_t  hf_uuid = {2, {UUID_SERVCLASS_HF_HANDSFREE}};
static wiced_bt_uuid_t  hs_uuid = {2, {UUID_SERVCLASS_HEADSET}};
static hfp_ag_session_cb_t* sdp_p_scb;

/******************************************************
*               Function Declarations
******************************************************/

/* declare sdp callback functions */
static void hfp_ag_sdp_free_db( hfp_ag_session_cb_t *p_scb );
static BOOLEAN hfp_ag_sdp_find_attr( hfp_ag_session_cb_t *p_scb );

/*
 * SDP callback function.
 */
static void hfp_ag_sdp_cback( uint16_t sdp_status )
{
    uint16_t                event;
    hfp_ag_session_cb_t     *p_scb = sdp_p_scb;

    WICED_BT_TRACE( "hci_control_ag_sdp_cback status:0x%x, p_scb %x\n", sdp_status, p_scb );

    /* set event according to int/acp */
    if ( !p_scb->b_is_initiator )
    {
        if ( ( sdp_status == WICED_BT_SDP_SUCCESS ) || ( sdp_status == WICED_BT_SDP_DB_FULL ) )
        {
            hfp_ag_sdp_find_attr ( p_scb );
        }
    }
    else
    {
        if ( ( sdp_status == WICED_BT_SDP_SUCCESS ) || ( sdp_status == WICED_BT_SDP_DB_FULL ) )
        {
            if ( hfp_ag_sdp_find_attr( p_scb ) )
            {
                hfp_ag_rfcomm_do_open( p_scb );
            }
            else
            {
                /* reopen server and notify app of the failure */
                hfp_ag_rfcomm_start_server( p_scb );
                hfp_ag_process_open_callback( p_scb, HCI_CONTROL_HF_STATUS_FAIL_SDP );
            }
        }
        else
        {
            /* reopen server and notify app of the failure */
            hfp_ag_rfcomm_start_server(p_scb);
            hfp_ag_process_open_callback(p_scb, HCI_CONTROL_HF_STATUS_FAIL_SDP);
        }
    }
    hfp_ag_sdp_free_db( p_scb );
}

/*
 * Process SDP discovery results to find requested attributes for requested service.
 * Returns TRUE if results found, FALSE otherwise.
 */
BOOLEAN hfp_ag_sdp_find_attr( hfp_ag_session_cb_t *p_scb )
{
    wiced_bt_sdp_discovery_record_t     *p_rec = ( wiced_bt_sdp_discovery_record_t * ) NULL;
    wiced_bt_sdp_protocol_elem_t        pe;
    wiced_bt_sdp_discovery_attribute_t  *p_attr;
    BOOLEAN                             result = WICED_TRUE;
    wiced_bt_uuid_t                     uuid_list;

    if(p_scb->hf_profile_uuid == UUID_SERVCLASS_HF_HANDSFREE)
    {
        WICED_BT_TRACE( "Looking for HFP service\n" );
        uuid_list.len       = LEN_UUID_16;
        uuid_list.uu.uuid16 = p_scb->hf_profile_uuid;
    }
    else
    {
        WICED_BT_TRACE( "Looking for HSP service\n" );
        uuid_list.len       = LEN_UUID_16;
        uuid_list.uu.uuid16 = p_scb->hf_profile_uuid;
    }

    p_rec = wiced_bt_sdp_find_service_uuid_in_db( p_scb->p_sdp_discovery_db, &uuid_list, p_rec );
    if ( p_rec == NULL )
    {
        WICED_BT_TRACE( "hci_control_ag_sdp_find_attr( ) - could not find AG service\n" );
        return ( WICED_FALSE );
    }

    /*** Look up the server channel number in the protocol list element ***/
    if ( wiced_bt_sdp_find_protocol_list_elem_in_rec( p_rec, UUID_PROTOCOL_RFCOMM, &pe ) )
    {
        WICED_BT_TRACE( "hci_control_ag_sdp_find_attr - num of proto elements -RFCOMM =0x%x\n",  pe.num_params );
        if ( pe.num_params > 0 )
        {
            p_scb->hf_scn = ( uint8_t )pe.params[0];
            WICED_BT_TRACE( "hci_control_ag_sdp_find_attr - found SCN in SDP record. SCN=0x%x\n", p_scb->hf_scn );
        }
        else
            result = WICED_FALSE;
    }
    else
    {
        result = WICED_FALSE;
    }

    /* get HFP supported features ( attribute ID 0x0311 ) */
    if ( p_scb->hf_profile_uuid == UUID_SERVCLASS_HF_HANDSFREE)
    {
        if ( ( p_attr = wiced_bt_sdp_find_attribute_in_rec( p_rec, 0x0311 ) ) != NULL )
        {
            /* Found attribute. Get value. but do not overwrite peer_feature if we already received +BRSF */
            if ( p_scb->hf_features == 0 )
                p_scb->hf_features = p_attr->attr_value.v.u16;
        }

        if ( wiced_bt_sdp_find_profile_version_in_rec( p_rec, UUID_SERVCLASS_HF_HANDSFREE, &p_scb->hf_version ) )
        {
            WICED_BT_TRACE( "HF device profile version: 0x%x\n", p_scb->hf_version );
        }
    }

    return result;
}


/*
 * Do service discovery.
 */
void hfp_ag_sdp_start_discovery( hfp_ag_session_cb_t *p_scb )
{
    uint16_t        attr_list[4];
    uint8_t         num_attr;
    wiced_bt_uuid_t uuid_list;

    /* initiator - get proto list and features */
    if ( p_scb->b_is_initiator )
    {
        attr_list[0] = ATTR_ID_SERVICE_CLASS_ID_LIST;
        attr_list[1] = ATTR_ID_PROTOCOL_DESC_LIST;
        attr_list[2] = ATTR_ID_BT_PROFILE_DESC_LIST;
        attr_list[3] = ATTR_ID_SUPPORTED_FEATURES;
        num_attr = 4;
    }
    /* HFP acceptor; get features */
    else
    {
        attr_list[0] = ATTR_ID_SERVICE_CLASS_ID_LIST;
        attr_list[1] = ATTR_ID_BT_PROFILE_DESC_LIST;
        attr_list[2] = ATTR_ID_SUPPORTED_FEATURES;
        num_attr = 3;
    }

    /* allocate buffer for sdp database */
    p_scb->p_sdp_discovery_db = ( wiced_bt_sdp_discovery_db_t * ) wiced_bt_get_buffer( WICED_BUFF_MAX_SIZE );

    uuid_list.len       = LEN_UUID_16;
    uuid_list.uu.uuid16 = p_scb->hf_profile_uuid;
    /* set up service discovery database; attr happens to be attr_list len */
    wiced_bt_sdp_init_discovery_db( p_scb->p_sdp_discovery_db, WICED_BUFF_MAX_SIZE, 1, &uuid_list, num_attr, attr_list );

    WICED_BT_TRACE("  initiate service discovery app_handle = %x\n",p_scb->app_handle);

    /* initiate service discovery */
    if ( !wiced_bt_sdp_service_search_attribute_request( p_scb->hf_addr, p_scb->p_sdp_discovery_db, hfp_ag_sdp_cback ) )
    {
        WICED_BT_TRACE("hfp_ag_sdp_start_discovery: wiced_bt_sdp_service_search_attribute_request fail\n");
        /* Service discovery not initiated - free discover db, reopen server, tell app  */
        hfp_ag_sdp_free_db( p_scb );

        if ( p_scb->b_is_initiator )
        {
            hfp_ag_rfcomm_start_server( p_scb );
            hfp_ag_process_open_callback( p_scb, HCI_CONTROL_HF_STATUS_FAIL_SDP );
        }
    }
    else
    {
        /* save the p_scb in this service discovery, sdp_cback need this */
        sdp_p_scb = p_scb;
    }
}

/*
 * Free discovery database.
 */
void hfp_ag_sdp_free_db( hfp_ag_session_cb_t *p_scb )
{
    if ( p_scb->p_sdp_discovery_db != NULL )
    {
        wiced_bt_free_buffer( p_scb->p_sdp_discovery_db );
        p_scb->p_sdp_discovery_db = NULL;
    }
}
