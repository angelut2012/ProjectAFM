clear
load im_buffer_raw

% if ~isempty(h_rect)
%     if h_rect.isvalid()~=1
%         h_rect.delete()
%     end
% end
XL=5000
YL=6000
DX=7000
DY=6500
position=[XL,YL,DX,DY]
position=AFM_scan_set_ROI(im_buffer,position)