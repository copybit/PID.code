switch exampleID
    case 0 % Deafult example - Simple code with noise and filtering
    
        t=Dt:Dt:Tsim;  
        
        uc=uc(Lp+2:end);
        yc=yc(Lp+2:end);
        rc=rc(Lp+2:end);
        t=t(1:end-Lp-1);
        
        figure(1);
        font_size=14;
        subplot(2,1,1)
        plot(t',rc+y0,'r','LineWidth',2);hold on;
        plot(t',yc+y0,'k','LineWidth',2);
        
        
        legend('Setpoint','Process output','Location','southeast','FontSize',font_size)
        ylabel('Process output','FontSize',font_size)
        xlabel('Time (s)','FontSize',font_size)
        grid
        
        ax = gca;
        ax.YAxis.FontSize = font_size;
        ax.XAxis.FontSize = font_size;
        xticks(0:2:Tsim);
        
        ymax=max([yc]);ymin=min([yc]);
        yMargin = 0.1 * (ymax - ymin); 
        ylim([ymin - yMargin, ymax + yMargin]);
        
        subplot(2,1,2)
        plot(t',uc,'k','LineWidth',2);hold on;
        
        legend('Control signal','Location','southeast','FontSize',font_size)
        ylabel('Controller output','FontSize',font_size)
        xlabel('Time (s)','FontSize',font_size);
        grid
        
        ax = gca;
        ax.YAxis.FontSize = font_size;
        ax.XAxis.FontSize = font_size;
        xticks(0:2:Tsim);
        
        umax=max([uc]);umin=min([uc]);
        yMargin = 0.1 * (umax - umin); 
        ylim([umin - yMargin, umax + yMargin]);

    case 1 % Example 1 - From man to auto

        t=Dt:Dt:Tsim;  
        
        uc=uc(Lp+2:end);
        yc=yc(Lp+2:end);
        rc=rc(Lp+2:end);
        t=t(1:end-Lp-1);
        
        figure(1)
        font_size=14;
        subplot(2,1,1)
        plot(t',rc+y0,'r','LineWidth',2);hold on;
        plot(t',yc+y0,'k','LineWidth',2);grid;
        legend('Setpoint','y','Location','southeast','FontSize',font_size)
        ylabel('Process output','FontSize',font_size)
        xlabel('Time (s)','FontSize',font_size)
        
        ax = gca;
        ax.YAxis.FontSize = font_size;
        ax.XAxis.FontSize = font_size;
        xticks(0:2:Tsim);
        
        yMargin = 0.1 * (max(yc) - min(yc));  
        ylim([min(yc) - yMargin, max(yc) + yMargin]);
        
        subplot(2,1,2)
        plot(t',uc,'k','LineWidth',2);hold on;
        grid
        legend('u','Location','southeast','FontSize',font_size)
        ylabel('Controller output','FontSize',font_size)
        xlabel('Time (s)','FontSize',font_size);
        
        ax = gca;
        ax.YAxis.FontSize = font_size;
        ax.XAxis.FontSize = font_size;
        xticks(0:2:Tsim);
        
        yMargin = 0.1 * (max(uc) - min(uc));  
        ylim([min(uc) - yMargin, max(uc) + yMargin]);

    case 2 % Example 2 - P, PD control and rate limitation
        t=Dt:Dt:Tsim;  
        
        uc=uc(Lp+2:end);
        yc=yc(Lp+2:end);
        rc=rc(Lp+2:end);
        t=t(1:end-Lp-1);
        
        figure(1)
        font_size=14;
        subplot(2,1,1)
        plot(t',rc+y0,'r','LineWidth',2);hold on;
        plot(t',yc+y0,'k','LineWidth',2);
        legend('Setpoint','Process output','Location','northeast','FontSize',font_size)
        ylabel('Process output','FontSize',font_size)
        xlabel('Time (s)','FontSize',font_size)
        grid
        
        ax = gca;
        ax.YAxis.FontSize = font_size;
        ax.XAxis.FontSize = font_size;
        
        yMargin = 0.1 * (max(yc) - min(yc));  
        ylim([min(yc) - yMargin, max(yc) + yMargin]);
        
        subplot(2,1,2)
        plot(t',uc,'k-','LineWidth',2);hold on;
        legend('Control signal','Location','northeast','FontSize',font_size)
        ylabel('Controller output','FontSize',font_size)
        xlabel('Time (s)','FontSize',font_size);
        
        ax = gca;
        ax.YAxis.FontSize = font_size;
        ax.XAxis.FontSize = font_size;
        
        yMargin = 0.1 * (max(uc) - min(uc));  
        ylim([min(uc) - yMargin, max(uc) + yMargin]);
        grid

    case 3 % Example 3 - Set-point weighting
        t=Dt:Dt:Tsim;  
        
        uc1=uc1(Lp+2:end);uc2=uc2(Lp+2:end);uc3=uc3(Lp+2:end);
        yc1=yc1(Lp+2:end);yc2=yc2(Lp+2:end);yc3=yc3(Lp+2:end);
        rc=rc(Lp+2:end);
        t=t(1:end-Lp-1);
        
        figure(1)
        font_size=14;
        subplot(2,1,1)
        plot(t',rc+y0,'r','LineWidth',2);hold on;
        plot(t',yc1+y0,'k--','LineWidth',2);
        plot(t',yc2+y0,'k:','LineWidth',2);
        plot(t',yc3+y0,'k','LineWidth',2);
        
        legend('Setpoint','b=1','b=0.5','b=0','Location','southeast','FontSize',font_size)
        ylabel('Process output','FontSize',font_size)
        xlabel('Time (s)','FontSize',font_size)
        grid
        
        ax = gca;
        ax.YAxis.FontSize = font_size;
        ax.XAxis.FontSize = font_size;
        xticks(0:2:Tsim);
        
        ymax=max([yc1;yc2;yc3]);ymin=min([yc1;yc2;yc3]);
        yMargin = 0.1 * (ymax - ymin); 
        ylim([ymin - yMargin, ymax + yMargin]);
        
        subplot(2,1,2)
        plot(t',uc1,'k--','LineWidth',2);hold on;
        plot(t',uc2,'k:','LineWidth',2);
        plot(t',uc3,'k','LineWidth',2);
        
        legend('b=1','b=0.5','b=0','Location','southeast','FontSize',font_size)
        ylabel('Controller output','FontSize',font_size)
        xlabel('Time (s)','FontSize',font_size);
        grid
        
        ax = gca;
        ax.YAxis.FontSize = font_size;
        ax.XAxis.FontSize = font_size;
        xticks(0:2:Tsim);
        
        umax=max([uc1;uc2;uc3]);umin=min([uc1;uc2;uc3]);
        yMargin = 0.1 * (umax - umin); 
        ylim([umin - yMargin, umax + yMargin]);        

    case 4 % Example 4 - Feedforward and anti-windup

        t=Dt:Dt:Tsim;  
        
        uc1=uc1(Lp+2:end);uc2=uc2(Lp+2:end);uc3=uc3(Lp+2:end);uc4=uc4(Lp+2:end);
        y1=y1(Lp+2:end);y2=y2(Lp+2:end);y3=y3(Lp+2:end);y4=y4(Lp+2:end);
        yd=yd(Lp+2:end);
        rc=rc(Lp+2:end);
        t=t(1:end-Lp-1);
        
        figure(1)
        font_size=14;
        subplot(2,1,1)
        plot(t',rc+y0,'r','LineWidth',2);hold on;
        plot(t',y1+y0,'b','LineWidth',2);
        plot(t',y2+y0,'k:','LineWidth',2);
        plot(t',y3+y0,'k--','LineWidth',2);
        plot(t',y4+y0,'k','LineWidth',2);
        legend('Setpoint','No saturation','No anti-windup','Control signal clamping','Back calculation','Location','northeast','FontSize',font_size)
        ylabel('Process output','FontSize',font_size)
        xlabel('Time (s)','FontSize',font_size)
        grid on
        
        ax = gca;
        ax.YAxis.FontSize = font_size;
        ax.XAxis.FontSize = font_size;

        ymax=max([y1;y2;y3;y4]);ymin=min([y1;y2;y3;y4]);
        yMargin = 0.1 * (ymax - ymin); 
        ylim([ymin - yMargin, ymax + yMargin]);        
        
        subplot(2,1,2)
        plot(t',uc1,'b','LineWidth',2);hold on;
        plot(t',uc2,'k:','LineWidth',2);
        plot(t',uc3,'k--','LineWidth',2);
        plot(t',uc4,'k','LineWidth',2);
        grid on
        
        legend('No saturation','No anti-windup','Control signal clamping','Back calculation','Location','northeast','FontSize',font_size)
        ylabel('Control signal','FontSize',font_size)
        xlabel('Time (s)','FontSize',font_size);
        
        ax = gca;
        ax.YAxis.FontSize = font_size;
        ax.XAxis.FontSize = font_size;

        ymax=max([uc1;uc2;uc3;uc4]);ymin=min([uc1;uc2;uc3;uc4]);
        yMargin = 0.1 * (ymax - ymin); 
        ylim([ymin - yMargin, ymax + yMargin]);          

    case 5 % Example 5 - Filtering measurement noise

        t=Dt:Dt:Tsim;  

        uc1=uc1(Lp+2:end);uc2=uc2(Lp+2:end);uc3=uc3(Lp+2:end);
        yc1=yc1(Lp+2:end);yc2=yc2(Lp+2:end);yc3=yc3(Lp+2:end);
        rc=rc(Lp+2:end);
        t=t(1:end-Lp-1);
        
        figure(1)
        font_size=14;
        subplot(2,1,1)
        plot(t',rc+y0,'r','LineWidth',2);hold on;
        plot(t',yc1+y0,'k','LineWidth',2);
        plot(t',yc2+y0,'b','LineWidth',2);
        plot(t',yc3+y0,'g','LineWidth',2);
        
        
        legend('Setpoint','No filter','Filter T_f=0.1*T_i','Filter T_f=0.01*T_i','Location','southeast','FontSize',font_size);
        ylabel('Process output','FontSize',font_size)
        xlabel('Time (s)','FontSize',font_size)
        grid
        
        ax = gca;
        ax.YAxis.FontSize = font_size;
        ax.XAxis.FontSize = font_size;
        xticks(0:2:Tsim);
        
        ymax=max([yc1;yc2]);ymin=min([yc1;yc2]);
        yMargin = 0.1 * (ymax - ymin); 
        ylim([ymin - yMargin, ymax + yMargin]);
        
        subplot(2,1,2)
        plot(t',uc1,'k','LineWidth',2);hold on;
        plot(t',uc2,'b','LineWidth',2);
        plot(t',uc3,'g','LineWidth',2);
        
        legend('No filter','Filter T_f=0.1*T_i','Filter T_f=0.01*T_i','Location','southeast','FontSize',font_size);
        ylabel('Controller output','FontSize',font_size)
        xlabel('Time (s)','FontSize',font_size);
        grid
        
        ax = gca;
        ax.YAxis.FontSize = font_size;
        ax.XAxis.FontSize = font_size;
        xticks(0:2:Tsim);
        
        umax=max([uc1;uc2]);umin=min([uc1;uc2]);
        yMargin = 0.1 * (umax - umin); 
        ylim([umin - yMargin, umax + yMargin]);

    case 6 % Example 6 - Adaptive control of tank level

        t=Dt:Dt:Tsim;  
        
        uc=uc(Lp+1:end-1);
        yc=yc(Lp+2:end);
        rc=rc(Lp+1:end-1);
        t=t(1:end-Lp-1);
        
        figure(1)
        font_size=14;
        subplot(3,1,1)
        stairs(t',rc,'r','LineWidth',2);hold on
        plot(t',yc,'k','LineWidth',2);
        
        subplot(3,1,2)
        plot(t',uc,'k','LineWidth',2);hold on;   

        ucs=ucs(Lp+1:end-1);
        ycs=ycs(Lp+2:end);
        rcs=rcs(Lp+1:end-1);
        
        figure(1)
        font_size=14;
        subplot(3,1,1)
        plot(t',ycs,'k:','LineWidth',2);
        legend('Setpoint','Multiple controllers','Single controller','Location','southeast','FontSize',font_size)
        ylabel('Tank level (cm)','FontSize',font_size)
        xlabel('Time (s)','FontSize',font_size)
        grid
        
        ax = gca;
        ax.YAxis.FontSize = font_size;
        ax.XAxis.FontSize = font_size;
        
        yMargin = 0.1 * (max(ycs) - min(ycs));  % Margen del 10% del rango de Y
        ylim([min(ycs) - yMargin, max(ycs) + yMargin]);
        
        subplot(3,1,2)
        plot(t',uc,'k:','LineWidth',2);hold on;
        legend('Multiple controllers','Single controller','Location','southeast','FontSize',font_size)
        ylabel('Flow rate (cm^3/s)','FontSize',font_size)
        xlabel('Time (s)','FontSize',font_size);
        
        ax = gca;
        ax.YAxis.FontSize = font_size;
        ax.XAxis.FontSize = font_size;
        grid on
        
        yMargin = 0.1 * (max(ucs) - min(ucs));  % Margen del 10% del rango de Y
        ylim([min(ucs) - yMargin, max(ucs) + yMargin]);
        
        
        subplot(3,1,3)
        modess=modess(Lp+1:end-1);
        stairs(t',modess,'k','LineWidth',2);
        ylabel('Nº active controller')
        xlabel('Time (s)','FontSize',font_size);
        legend('Controller active','FontSize',font_size)
        grid on
        
        ax = gca;
        ax.YAxis.FontSize = font_size;
        ax.XAxis.FontSize = font_size;
        grid on
        
        yMargin = 0.1 * (max(modess) - min(modess));  % Margen del 10% del rango de Y
        ylim([min(modess) - yMargin, max(modess) + yMargin]);        

    case 7 % Example 7 - Selectors

        t=Dt:Dt:Tsim;   
        
        uc=uc(Lp+1:end);
        yc1=yc1(Lp+1:end);
        yc2=yc2(Lp+1:end);
        rc1=rc1(Lp+1:end);
        rc2=rc2(Lp+1:end);
        t=t(1:end-Lp);
        uc1=uc1(Lp+1:end);
        uc2=uc2(Lp+1:end);
        v=v(Lp+1:end-1);
        
        %% Indexes for shading areas and vertical lines
        auxu=uc1-uc2;
        ii=find(abs(auxu)<0.000001);
        
        figure
        font_size=14;
        
        subplot(4,1,1)
        plot(t',rc1,'r','LineWidth',2);hold on;
        plot(t',rc2,'r:','LineWidth',2);
        plot(t',yc1,'b','LineWidth',2);
        plot(t',yc2,'b:','LineWidth',2);
        ylabel('Process outputs','FontSize',font_size)
        grid on
        xlabel('Time (s)','FontSize',font_size);
        
        %%% Shadow areas
        ax = gca;
        ax.YLim(2)=0.7;
        ax.YLim(1)=-0.7;
        
        % Left
        x1=0;x2=t(ii(2));y1=min(ax.YLim(1));y2=max(ax.YLim(2));
        r = rectangle('Position',[x1,y1,x2-x1,y2-y1], ...
                      'FaceColor',[0.9 0.9 0.9], 'EdgeColor','none', 'FaceAlpha', 0.9);
        line([t(ii(2)), t(ii(2))],[y1 y2],'Color','k', 'LineStyle',':');
        uistack(r,'bottom')
        
        % Middle
        x1=t(ii(2));x2=t(ii(3));y1=min(ax.YLim(1));y2=max(ax.YLim(2));
        r = rectangle('Position',[x1,y1,x2-x1,y2-y1], ...
                      'FaceColor',[0.9 0.9 0.9], 'EdgeColor','none', 'FaceAlpha', 0.5);
        line([t(ii(3)), t(ii(3))],[y1 y2],'Color','k', 'LineStyle',':');
        uistack(r,'bottom')
        
        % Right
        x1=t(ii(3));x2=t(end);y1=min(ax.YLim(1));y2=max(ax.YLim(2));
        r = rectangle('Position',[x1,y1,x2-x1,y2-y1], ...
                      'FaceColor',[0.9 0.9 0.9], 'EdgeColor','none', 'FaceAlpha', 0.9);
        uistack(r,'bottom')
        
        legend('r1','r2','y1','y2','Location','southeast','FontSize',font_size)
        
        yl = ylim;
        yticks(yl(1):0.3:yl(2));
        ax.YAxis.FontSize = font_size;
        ax.XAxis.FontSize = font_size;
        grid on
        
        subplot(4,1,2)
        plot(t',uc1,'b','LineWidth',2);hold on
        plot(t',uc2,'b:','LineWidth',2)
        plot(t',uc,'m--','LineWidth',2);
        grid on
        ylabel('Control signals','FontSize',font_size)
        xlabel('Time (s)','FontSize',font_size);
        
        %%% Shadow areas
        ax = gca;
        
        % Left
        x1=0;x2=t(ii(2));y1=min(ax.YLim(1));y2=max(ax.YLim(2));
        r = rectangle('Position',[x1,y1,x2-x1,y2-y1], ...
                      'FaceColor',[0.9 0.9 0.9], 'EdgeColor','none', 'FaceAlpha', 0.9);
        line([t(ii(2)), t(ii(2))],[y1 y2],'Color','k', 'LineStyle',':');
        uistack(r,'bottom')
        
        % Middle
        x1=t(ii(2));x2=t(ii(3));y1=min(ax.YLim(1));y2=max(ax.YLim(2));
        r = rectangle('Position',[x1,y1,x2-x1,y2-y1], ...
                      'FaceColor',[0.9 0.9 0.9], 'EdgeColor','none', 'FaceAlpha', 0.5);
        line([t(ii(3)), t(ii(3))],[y1 y2],'Color','k', 'LineStyle',':');
        uistack(r,'bottom')
        
        % Right
        x1=t(ii(3));x2=t(end);y1=min(ax.YLim(1));y2=max(ax.YLim(2));
        r = rectangle('Position',[x1,y1,x2-x1,y2-y1], ...
                      'FaceColor',[0.9 0.9 0.9], 'EdgeColor','none', 'FaceAlpha', 0.9);
        uistack(r,'bottom')
        
        legend('uc1','uc2','u','Location','southeast','FontSize',font_size);
        
        ax.YAxis.FontSize = font_size;
        ax.XAxis.FontSize = font_size;
        
        subplot(4,1,3)
        
        plot(t',uc1-uc2,'b','LineWidth',2);hold on
        line([0, t(end)],[0 0], 'Color','k', 'LineStyle',':','LineWidth',2)
        grid on
        
        %%% Shadow areas
        ax = gca;
        ax.YLim(1)=-0.003;
        
        % Left
        x1=0;x2=t(ii(2));y1=min(ax.YLim(1));y2=max(ax.YLim(2));
        r = rectangle('Position',[x1,y1,x2-x1,y2-y1], ...
                      'FaceColor',[0.9 0.9 0.9], 'EdgeColor','none', 'FaceAlpha', 0.9);
        line([t(ii(2)), t(ii(2))],[y1 y2],'Color','k', 'LineStyle',':');
        
        text(ax, t(ii(2))/2, y1+(y2-y1)/1.8, 'C2', 'FontSize', 14);
        
        uistack(r,'bottom')
        
        % Middle
        x1=t(ii(2));x2=t(ii(3));y1=min(ax.YLim(1));y2=max(ax.YLim(2));
        r = rectangle('Position',[x1,y1,x2-x1,y2-y1], ...
                      'FaceColor',[0.9 0.9 0.9], 'EdgeColor','none', 'FaceAlpha', 0.5);
        line([t(ii(3)), t(ii(3))],[y1 y2],'Color','k', 'LineStyle',':');
        
        text(ax, t(end)/2, y1+(y2-y1)/1.8, 'C1', 'FontSize', 14);
        
        uistack(r,'bottom')
        
        % Right
        x1=t(ii(3));x2=t(end);y1=min(ax.YLim(1));y2=max(ax.YLim(2));
        r = rectangle('Position',[x1,y1,x2-x1,y2-y1], ...
                      'FaceColor',[0.9 0.9 0.9], 'EdgeColor','none', 'FaceAlpha', 0.9);
        
        text(ax, t(ii(3))+(t(end)-t(ii(3)))/2-5, y1+(y2-y1)/1.8, 'C2', 'FontSize', 14);
        
        uistack(r,'bottom')
        
        legend('uc1-uc2','FontSize',font_size)
        ylabel('uc1-uc2','FontSize',font_size)
        xlabel('Time (s)','FontSize',font_size);
        
        ax = gca;
        ax.YAxis.FontSize = font_size;
        ax.XAxis.FontSize = font_size;
        
        %% v
        subplot(4,1,4)
        plot(t',v,'LineWidth',2);grid
        legend('v','FontSize',font_size)
        ylabel('v','FontSize',font_size)
        xlabel('Time (s)','FontSize',font_size);
        
        ax = gca;
        ax.YLim(1)=-0.2;
        ax.YAxis.FontSize = font_size;
        ax.XAxis.FontSize = font_size;
        grid on
        

    otherwise % Deafult example 
        error('Wrong Example ID');         
end

tightfig;
