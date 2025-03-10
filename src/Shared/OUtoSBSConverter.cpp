#include "OUtoSBSConverter.h"

#include "Util.h"

ID3D11Texture2D* OUtoSBSConverter::GetTexture() const
{
    return (m_MultiGPUTexSBSTarget != nullptr) ? m_MultiGPUTexSBSTarget.Get() : m_TexSBS.Get();
}

Vector2Int OUtoSBSConverter::GetTextureSizeSBS() const
{
    return m_TextSizeSBS;
}

HRESULT OUtoSBSConverter::Convert(ID3D11Device* device, ID3D11DeviceContext* device_context, ID3D11Device* multi_gpu_device, ID3D11DeviceContext* multi_gpu_device_context, 
                                  ID3D11Texture2D* tex_source, int tex_source_width, int tex_source_height, int crop_x, int crop_y, int crop_width, int crop_height)
{
    Vector2Int sbs_size(crop_width * 2, crop_height / 2);

    //Resource setup on first time or when dimensions changed
    if ( (m_TexSBS == nullptr) || (sbs_size != m_TextSizeSBS) )
    {
        m_TextSizeSBS = sbs_size;

        //Delete old resources if they exist
        CleanRefs();

        //Get source desc to match format (we assume it doesn't change unexpectedly to avoid doing this every call)
        D3D11_TEXTURE2D_DESC tex_source_desc;
        tex_source->GetDesc(&tex_source_desc);

        //Create texture
        D3D11_TEXTURE2D_DESC TexD;
        RtlZeroMemory(&TexD, sizeof(D3D11_TEXTURE2D_DESC));
        TexD.Width  = m_TextSizeSBS.x;
        TexD.Height = m_TextSizeSBS.y;
        TexD.MipLevels = 1;
        TexD.ArraySize = 1;
        TexD.Format = tex_source_desc.Format;
        TexD.SampleDesc.Count = 1;
        TexD.Usage = D3D11_USAGE_DEFAULT;
        TexD.BindFlags = D3D11_BIND_SHADER_RESOURCE;
        TexD.CPUAccessFlags = 0;
        TexD.MiscFlags = 0;

        HRESULT hr = device->CreateTexture2D(&TexD, nullptr, &m_TexSBS);

        if (FAILED(hr))
            return hr;

        //Create textures for multi-gpu processing if needed
        if (multi_gpu_device != nullptr) 
        {
            //Staging texture
            TexD.Usage = D3D11_USAGE_STAGING;
            TexD.BindFlags = 0;
            TexD.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
            TexD.MiscFlags = 0;

            hr = device->CreateTexture2D(&TexD, nullptr, &m_MultiGPUTexSBSStaging);

            if (FAILED(hr))
                return hr;

            //Copy-target texture
            TexD.Usage = D3D11_USAGE_DYNAMIC;
            TexD.BindFlags = D3D11_BIND_SHADER_RESOURCE;
            TexD.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            TexD.MiscFlags = 0;

            hr = multi_gpu_device->CreateTexture2D(&TexD, nullptr, &m_MultiGPUTexSBSTarget);

            if (FAILED(hr))
                return hr;
        }
    }

    //Copy top and bottom half of the cropped region into the left and right halves of SBS texture
    D3D11_BOX source_region = {0};
    source_region.left   = clamp(crop_x, 0, tex_source_width);
    source_region.right  = clamp(crop_x + crop_width, 0, tex_source_width);
    source_region.top    = clamp(crop_y, 0, tex_source_height);
    source_region.bottom = clamp(crop_y + m_TextSizeSBS.y, 0, tex_source_height);
    source_region.front  = 0;
    source_region.back   = 1;

    device_context->CopySubresourceRegion(m_TexSBS.Get(), 0, 0, 0, 0, tex_source, 0, &source_region);          //Top -> Left

    source_region.top    = source_region.bottom;
    source_region.bottom = clamp((int)source_region.top + m_TextSizeSBS.y, 0, tex_source_height);

    device_context->CopySubresourceRegion(m_TexSBS.Get(), 0, crop_width, 0, 0, tex_source, 0, &source_region); //Bottom -> Right

    //If set up for multi-gpu processing, copy the texture over
    if (m_MultiGPUTexSBSTarget != nullptr)
    {
        //Same as in OutputManager::RefreshOpenVROverlayTexture
        device_context->CopyResource(m_MultiGPUTexSBSStaging.Get(), m_TexSBS.Get());

        D3D11_MAPPED_SUBRESOURCE mapped_resource_staging;
        RtlZeroMemory(&mapped_resource_staging, sizeof(D3D11_MAPPED_SUBRESOURCE));
        HRESULT hr = device_context->Map(m_MultiGPUTexSBSStaging.Get(), 0, D3D11_MAP_READ, 0, &mapped_resource_staging);

        if (FAILED(hr))
            return hr;

        D3D11_MAPPED_SUBRESOURCE mapped_resource_target;
        RtlZeroMemory(&mapped_resource_target, sizeof(D3D11_MAPPED_SUBRESOURCE));
        hr = multi_gpu_device_context->Map(m_MultiGPUTexSBSTarget.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped_resource_target);

        if (FAILED(hr))
            return hr;

        memcpy(mapped_resource_target.pData, mapped_resource_staging.pData, (size_t)m_TextSizeSBS.y * mapped_resource_staging.RowPitch);

        device_context->Unmap(m_MultiGPUTexSBSStaging.Get(), 0);
        multi_gpu_device_context->Unmap(m_MultiGPUTexSBSTarget.Get(), 0);
    }

    return S_OK;
}

void OUtoSBSConverter::CleanRefs()
{
    m_TexSBS.Reset();
    m_MultiGPUTexSBSStaging.Reset();
    m_MultiGPUTexSBSTarget.Reset();
}
