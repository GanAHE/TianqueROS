// Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "common_internal.h"

#include <string.h>

static size_t ucdr_next_remaining_size(ucdrBuffer* ub, size_t bytes, size_t data_size);

// -------------------------------------------------------------------
//                   INTERNAL UTIL IMPLEMENTATIONS
// -------------------------------------------------------------------

bool ucdr_check_buffer_available_for(ucdrBuffer* ub, size_t bytes)
{
    return !ub->error && (ub->iterator + bytes <= ub->final);
}

bool ucdr_check_final_buffer_behavior(ucdrBuffer* ub, size_t data_size)
{
    if(!ub->error && ub->iterator + data_size > ub->final)
    {
        ub->error = (NULL != ub->on_full_buffer) ? ub->on_full_buffer(ub, ub ->args) : true;
    }

    return !ub->error;
}

size_t ucdr_next_remaining_size(ucdrBuffer* ub, size_t bytes, size_t data_size)
{
    size_t remaining = ucdr_buffer_remaining(ub);
    return (ub->iterator + bytes <= ub->final)
           ? bytes
           : remaining - (remaining % data_size);
}

size_t ucdr_check_final_buffer_behavior_array(ucdrBuffer* ub, size_t bytes, size_t data_size)
{
    if(!ub->error && ub->iterator + data_size > ub->final && bytes > 0)
    {
        ub->error = (NULL != ub->on_full_buffer) ? ub->on_full_buffer(ub, ub->args) : true;
    }

    return (!ub->error) ? ucdr_next_remaining_size(ub, bytes, data_size) : 0;
}

void ucdr_set_on_full_buffer_callback(ucdrBuffer* ub, OnFullBuffer on_full_buffer, void* args)
{
    ub->on_full_buffer = on_full_buffer;
    ub->args = args;
}

// -------------------------------------------------------------------
//                       PUBLIC IMPLEMENTATION
// -------------------------------------------------------------------
void ucdr_init_buffer(ucdrBuffer* ub, uint8_t* data, size_t size)
{
    ucdr_init_buffer_offset(ub, data, size, 0u);
}

void ucdr_init_buffer_offset(ucdrBuffer* ub, uint8_t* data, size_t size, size_t offset)
{
    ucdr_init_buffer_offset_endian(ub, data, size, offset, UCDR_MACHINE_ENDIANNESS);
}

void ucdr_init_buffer_offset_endian(ucdrBuffer* ub, uint8_t* data, size_t size, size_t offset, ucdrEndianness endianness)
{
    ub->init = data;
    ub->final = ub->init + size;
    ub->iterator = ub->init + offset;
    ub->last_data_size = 0u;
    ub->endianness = endianness;
    ub->error = false;
    ub->on_full_buffer = NULL;
    ub->args = NULL;
}


void ucdr_copy_buffer(ucdrBuffer* ub_dest, const ucdrBuffer* ub_source)
{
    memcpy(ub_dest, ub_source, sizeof(ucdrBuffer));
}

void ucdr_reset_buffer(ucdrBuffer* ub)
{
    ucdr_reset_buffer_offset(ub, 0u);
}

void ucdr_reset_buffer_offset(ucdrBuffer* ub, size_t offset)
{
    ub->iterator = ub->init + offset;
    ub->last_data_size = 0u;
    ub->error = false;
}

void ucdr_align_to(ucdrBuffer* ub, size_t size)
{
    ub->iterator += ucdr_buffer_alignment(ub, size);
    if(ub->iterator > ub->final)
    {
        ub->iterator = ub->final;
    }

    ub->last_data_size = (uint8_t)size;
}

size_t ucdr_alignment(size_t current_alignment, size_t data_size)
{
    return ((data_size - (current_alignment % data_size)) & (data_size - 1));
}

size_t ucdr_buffer_alignment(const ucdrBuffer* ub, size_t data_size)
{
    if(data_size > ub->last_data_size)
    {
        return (data_size - ((uint32_t)(ub->iterator - ub->init) % data_size)) & (data_size - 1);
    }

    return 0;
}

size_t ucdr_buffer_size(const ucdrBuffer* ub)
{
    return (size_t)(ub->final - ub->init);
}

size_t ucdr_buffer_length(const ucdrBuffer* ub)
{
    return (size_t)(ub->iterator - ub->init);
}

size_t ucdr_buffer_remaining(const ucdrBuffer* ub)
{
    return (size_t)(ub->final - ub->iterator);
}

ucdrEndianness ucdr_buffer_endianness(const ucdrBuffer* ub)
{
    return ub->endianness;
}

bool ucdr_buffer_has_error(const ucdrBuffer* ub)
{
    return ub->error;
}
