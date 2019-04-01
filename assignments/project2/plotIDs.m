function plotIDs(OOIs, txt_handles, extra_txt)
 %% Update text handles for plot IDs
 
 %% Function
  
 % Variables
 N_txt = length(txt_handles);
 N_nonzero_IDs = nnz(OOIs.IDs);
 nonzero_IDs = OOIs.IDs(OOIs.IDs~=0);
 nonzero_C_G = OOIs.Centers_G(OOIs.IDs~=0,:);
 
 % Sanit check
 assert(N_nonzero_IDs <= N_txt);
 
 for i = 1:N_txt
     
         if i <= N_nonzero_IDs
             % Set number
         set(txt_handles{i},'String', ...
            strcat(extra_txt, string(nonzero_IDs(i))), ...
             'Position', nonzero_C_G(i,:));    
         
         else
           % No nonzero_IDs left. Set empty string as text
             set(txt_handles{i},'String', '')
             
         end
 end
 

end