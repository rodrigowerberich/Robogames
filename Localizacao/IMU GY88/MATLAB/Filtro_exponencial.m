function [yt] = Filtro_exponencial(y)
    a = .9;
    yt(1) = y(1);
    yt(2) = a*yt(1)+(1-a)*y(2);
    yt(3) = a*yt(2)+(1-a)*y(3);
    for i=4:length(y)
        yt(i) = Media_movel_ponderada_exponencial(a,yt(i-3),y(i),y(i-1),y(i-2));
    end
%     plot(yt)
%     title('Saída')
%     xlabel('X')
%     ylabel('Y')
%     figure
%     plot(y)
%     title('Entrada')
%     xlabel('X')
%     ylabel('Y')
%     figure
end