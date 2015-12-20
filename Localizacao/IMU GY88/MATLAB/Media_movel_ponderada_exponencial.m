function [ m ] = Media_movel_ponderada_exponencial( a, m_3,v_k,v_1,v_2 )
%MEDIA_MOVEL_PONDERADA_EXPONENCIAL Summary of this function goes here
%   Detailed explanation goes here
m = a^3*m_3 + a^2*(1-a)*v_2+a*(1-a)*v_1+(1-a)*v_k;
end

