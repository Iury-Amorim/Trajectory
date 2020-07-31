function [G,F,E,M,Q,R] = smatrixH(Nu,N,A,B,C,D)
[~, nIN] = size(B);
%% Matrizes G, F, E e W
G = [];
F = [];
E = [];
aux = [];
for col = 1:N  %era pra ser Nu, mas tem a mudança nas linhas finais para resposta ao degrau
    for lin = 1:N
            if(lin-col)<0
                aux =[ aux; 0*C*A*B];
            else
                aux =[ aux; C*A^(lin-col)*B];
            end
    end
    G = [G aux];
    aux = [];
end

nG = Nu*nIN;
aux = 1:nIN;
aux = aux+(N-1)*nIN;
Gs  = zeros(N*nIN,nIN);
for i = 1:N-Nu
    index = aux-(i-1)*nIN;
    Gs = G(:,index)+Gs;
end
G = G(:,1:nG);
G(:,nG-nIN+1:end) = G(:,nG-nIN+1:end)+Gs;

[linG,colG] = size(G'*G);

for col = 1:N
    %Matriz F
    F =[F; C*A^col];
    
    %Matriz E
    E = [E; C*(A^(col-1))*D];
end

%% Matriz M
ordemSistema = nIN; 
M = [];
auxM = [];
for col = 1:Nu
    for lin = 1:Nu
            if(lin-col)<0
                auxM =[ auxM; zeros(ordemSistema)];
            elseif lin == col
                auxM =[ auxM; eye(ordemSistema)];
            elseif lin - col == 1
                auxM =[ auxM; -eye(ordemSistema)];
            else
                auxM =[ auxM; zeros(ordemSistema)];
            end            
    end
    M = [M auxM];
    auxM = [];
end

%% Matriz Q lambda
R = eye(linG,colG);

%% Matriz Q_delta
Q = [];
auxdel1 = [];
auxqdel2 = eye(nIN);
for col = 1:N
    for lin = 1:N
            if(lin==col)
            auxdel1 =[ auxdel1; auxqdel2*eye(nIN)];
            else
            auxdel1 =[ auxdel1; zeros(nIN)];   
            end
    end
    Q = [Q auxdel1];
    auxdel1 = [];
end
