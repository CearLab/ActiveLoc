        function [kconn, improveList] = checkKconnectivity(losTAB,agents_list,k)

            % kconn flag
            kconn = 0;
            fail = 0;
            improveList = [];

            % if it does not have at least 2 vertices return
            if size(agents_list,1) < k
                return;
            end

            % start from the full neighbourhood graph. if its connected
            % then proceed, otherwise it's not k-connected for sure

            % call util
            A = calcAdjacencyMatrix(losTAB,agents_list);
            [allConn, A] = checkConnectivity(A);

            if ~allConn
                return;
            else
                % now, we proceed by removing one k-tuple of vertices at a
                % time and check again connectivity

                % number of agents
                n = size(A,1);

                % pairs
                pairs = nchoosek(1:n,k-1);

                improveList = [];

                % cycle and check
                for i=1:size(pairs,1)
                    
                    % init
                    tmp = A;

                    % remove vertices
                    %removed one
                    tmpPair = pairs(i,:);
                    tmp(pairs(i,:),:) = [];
                    tmp(:,pairs(i,:)) = [];

                    isConn = checkConnectivity(tmp);

                    % if you find a non-connected submesh then you drop the
                    % search
                    if (~isConn)
                        fail = 1;
                        improveList(end+1:end+numel(agents_list(tmpPair,1))) = agents_list(tmpPair,1);
                    end

                end

                % set the flag
                if ~fail 
                    kconn = 1;
                end
            end

        end
