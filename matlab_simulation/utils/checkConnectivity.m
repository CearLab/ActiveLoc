function [isConn, Aout] = checkConnectivity(A)

            % if scalar exit
            if isscalar(A)
                isConn = 1;
                return;
            end
            
            % get number nodes
            n = size(A,1);            

            % if A^(n-1) has no zero elements than is connected
            Apow = A^(n-1);
            Azeros = find(Apow == 0);
            if isempty(Azeros)
                isConn = 1;
            else
                isConn = 0;
            end
            
            % out
            Aout = A;

        end