css_folder = css2400
css_platform_folder = hive_isp_css_2400_system
css_platform_folder_generated = hive_isp_css_2400_system_generated
base = base
runtime = runtime
kernels = isp/kernels

atomisp-objs := \
		atomisp_compat.o \
		atomisp_ioctl.o \
		atomisp_cmd.o \
		atomisp_fops.o \
		atomisp_subdev.o \
		atomisp_csi2.o \
		atomisp_tpg.o \
		atomisp_file.o \
		atomisp_v4l2.o \
		atomisp_acc.o \
		mmu/isp_mmu.o \
		mmu/sh_mmu_mrfld.o \
		mmu/sh_mmu_mfld.o \
		hmm/hmm.o \
		hmm/hmm_bo.o \
		hmm/hmm_bo_dev.o \
		hmm/hmm_vm.o \
		hrt/hive_isp_css_mm_hrt.o \
		hmm/hmm_reserved_pool.o \
		hmm/hmm_dynamic_pool.o
	#	hrt/device_access.o \
		hrt/memory_access.o \

atomisp-objs += \
		$(css_folder)/sh_css.o \
		$(css_folder)/sh_css_hrt.o \
		$(css_folder)/sh_css_params.o \
		$(css_folder)/sh_css_sp.o \
		$(css_folder)/sh_css_metrics.o \
		$(css_folder)/sh_css_firmware.o \
		$(css_folder)/sh_css_mipi.o \
		$(css_folder)/sh_css_param_shading.o \
		$(css_folder)/hive_isp_css_shared/host/tag.o \
		$(css_folder)/ia_css_memory_access.o \
		$(css_folder)/ia_css_device_access.o \
		$(css_folder)/sh_css_param_dvs.o \

atomisp-objs += $(css_folder)/$(css_platform_folder)/host/gdc.o \
		$(css_folder)/$(css_platform_folder)/host/irq.o \
		$(css_folder)/$(css_platform_folder)/host/mmu.o \
		$(css_folder)/$(css_platform_folder)/host/dma.o \
		$(css_folder)/$(css_platform_folder)/host/sp.o \
		$(css_folder)/$(css_platform_folder)/host/isp.o \
		$(css_folder)/$(css_platform_folder)/host/fifo_monitor.o \
		$(css_folder)/$(css_platform_folder)/host/gp_device.o \
		$(css_folder)/$(css_platform_folder)/host/input_system.o \
		$(css_folder)/$(css_platform_folder)/host/input_formatter.o \
        $(css_folder)/$(css_platform_folder)/host/input_system.o \

atomisp-objs += $(css_folder)/$(css_platform_folder_generated)/ia_css_isp_params.o \
		$(css_folder)/$(css_platform_folder_generated)/ia_css_isp_configs.o \
		$(css_folder)/$(css_platform_folder_generated)/ia_css_isp_states.o \

atomisp-objs +=	$(css_folder)/$(base)/circbuf/src/circbuf.o \
		$(css_folder)/$(base)/refcount/src/refcount.o \
		$(css_folder)/camera/pipe/src/pipe_binarydesc.o \
		$(css_folder)/camera/pipe/src/pipe_util.o \
		$(css_folder)/camera/pipe/src/pipe_stagedesc.o \
		$(css_folder)/camera/util/src/util.o \

atomisp-objs += $(css_folder)/$(runtime)/binary/src/binary.o \
		$(css_folder)/$(runtime)/bufq/src/bufq.o \
        $(css_folder)/$(runtime)/debug/src/ia_css_debug.o \
        $(css_folder)/$(runtime)/eventq/src/eventq.o \
        $(css_folder)/$(runtime)/isp_param/src/isp_param.o \
        $(css_folder)/$(runtime)/isys/src/isys_init.o \
        $(css_folder)/$(runtime)/isys/src/rx.o \
        $(css_folder)/$(runtime)/frame/src/frame.o \
        $(css_folder)/$(runtime)/rmgr/src/rmgr.o \
        $(css_folder)/$(runtime)/rmgr/src/rmgr_vbuf.o \
        $(css_folder)/$(runtime)/pipeline/src/pipeline.o \
        $(css_folder)/$(runtime)/event/src/event.o \
        $(css_folder)/$(runtime)/inputfifo/src/inputfifo.o \
        $(css_folder)/$(runtime)/spctrl/src/spctrl.o \
        $(css_folder)/$(runtime)/queue/src/queue.o \
        $(css_folder)/$(runtime)/queue/src/queue_access.o \
        $(css_folder)/$(runtime)/ifmtr/src/ifmtr.o \

atomisp-objs += $(css_folder)/$(kernels)/aa/aa_2/ia_css_aa2.host.o \
	   $(css_folder)/$(kernels)/anr/anr_2/ia_css_anr2.host.o \
       $(css_folder)/$(kernels)/anr/anr_2/ia_css_anr2_table.host.o \
       $(css_folder)/$(kernels)/anr/anr_1.0/ia_css_anr.host.o \
	   $(css_folder)/$(kernels)/bh/bh_2/ia_css_bh.host.o \
	   $(css_folder)/$(kernels)/bnr/bnr_1.0/ia_css_bnr.host.o \
	   $(css_folder)/$(kernels)/cnr/cnr_2/ia_css_cnr2.host.o \
       $(css_folder)/$(kernels)/cnr/cnr_1.0/ia_css_cnr.host.o \
	   $(css_folder)/$(kernels)/copy_output/copy_output_1.0/ia_css_copy_output.host.o \
	   $(css_folder)/$(kernels)/crop/crop_1.0/ia_css_crop.host.o \
	   $(css_folder)/$(kernels)/csc/csc_1.0/ia_css_csc.host.o \
	   $(css_folder)/$(kernels)/ctc/ctc_1.0/ia_css_ctc.host.o \
       $(css_folder)/$(kernels)/ctc/ctc_1.0/ia_css_ctc_table.host.o \
       $(css_folder)/$(kernels)/ctc/ctc_2/ia_css_ctc2.host.o \
	   $(css_folder)/$(kernels)/de/de_2/ia_css_de2.host.o \
       $(css_folder)/$(kernels)/de/de_1.0/ia_css_de.host.o \
       $(css_folder)/$(kernels)/dp/dp_1.0/ia_css_dp.host.o \
	   $(css_folder)/$(kernels)/dvs/dvs_1.0/ia_css_dvs.host.o \
	   $(css_folder)/$(kernels)/fpn/fpn_1.0/ia_css_fpn.host.o \
	   $(css_folder)/$(kernels)/gc/gc_1.0/ia_css_gc.host.o \
       $(css_folder)/$(kernels)/gc/gc_1.0/ia_css_gc_table.host.o \
       $(css_folder)/$(kernels)/gc/gc_2/ia_css_gc2.host.o \
       $(css_folder)/$(kernels)/gc/gc_2/ia_css_gc2_table.host.o \
       $(css_folder)/$(kernels)/iterator/iterator_1.0/ia_css_iterator.host.o \
       $(css_folder)/$(kernels)/macc/macc_1.0/ia_css_macc.host.o \
       $(css_folder)/$(kernels)/macc/macc_1.0/ia_css_macc_table.host.o \
       $(css_folder)/$(kernels)/ob/ob_1.0/ia_css_ob.host.o \
       $(css_folder)/$(kernels)/output/output_1.0/ia_css_output.host.o \
       $(css_folder)/$(kernels)/qplane/qplane_2/ia_css_qplane.host.o \
       $(css_folder)/$(kernels)/raw/raw_1.0/ia_css_raw.host.o \
       $(css_folder)/$(kernels)/raw_aa_binning/raw_aa_binning_1.0/ia_css_raa.host.o \
       $(css_folder)/$(kernels)/ref/ref_1.0/ia_css_ref.host.o \
       $(css_folder)/$(kernels)/sc/sc_1.0/ia_css_sc.host.o \
       $(css_folder)/$(kernels)/s3a/s3a_1.0/ia_css_s3a.host.o \
       $(css_folder)/$(kernels)/tnr/tnr_1.0/ia_css_tnr.host.o \
       $(css_folder)/$(kernels)/uds/uds_1.0/ia_css_uds.host.o \
       $(css_folder)/$(kernels)/vf/vf_1.0/ia_css_vf.host.o \
       $(css_folder)/$(kernels)/wb/wb_1.0/ia_css_wb.host.o \
       $(css_folder)/$(kernels)/xnr/xnr_1.0/ia_css_xnr.host.o \
       $(css_folder)/$(kernels)/xnr/xnr_1.0/ia_css_xnr_table.host.o \
       $(css_folder)/$(kernels)/ynr/ynr_1.0/ia_css_ynr.host.o \
       $(css_folder)/$(kernels)/ynr/ynr_2/ia_css_ynr2.host.o \

obj-$(CONFIG_VIDEO_ATOMISP) = atomisp.o

INCLUDES := -I$(srctree)/$(src)/include \
	    -I$(srctree)/$(src)/ \
	    -I$(src) \
	    -I$(src)/hrt \
	    -I$(srctree)/drivers/staging/intel_media/bc_video

INCLUDES += -I$(srctree)/$(src)/$(css_folder) \
		-I$(srctree)/$(src)/$(css_folder)/hrt \
		-I$(srctree)/$(src)/$(css_folder)/$(base)/circbuf/interface \
		-I$(srctree)/$(src)/$(css_folder)/$(base)/refcount/interface \
		-I$(srctree)/$(src)/$(css_folder)/camera/pipe/interface \
		-I$(srctree)/$(src)/$(css_folder)/camera/util/interface \
		-I$(srctree)/$(src)/$(css_folder)/$(css_platform_folder) \
	    -I$(srctree)/$(src)/$(css_folder)/$(css_platform_folder)/host \
	    -I$(srctree)/$(src)/$(css_folder)/$(css_platform_folder_generated) \
	    -I$(srctree)/$(src)/$(css_folder)/hive_isp_css_include \
	    -I$(srctree)/$(src)/$(css_folder)/hive_isp_css_include/host \
	    -I$(srctree)/$(src)/$(css_folder)/hive_isp_css_include/device_access \
	    -I$(srctree)/$(src)/$(css_folder)/hive_isp_css_include/memory_access \
	    -I$(srctree)/$(src)/$(css_folder)/hive_isp_css_shared \
	    -I$(srctree)/$(src)/$(css_folder)/hive_isp_css_shared/host \
	    -I$(srctree)/$(src)/$(css_folder)/$(kernels) \
	    -I$(srctree)/$(src)/$(css_folder)/$(kernels)/bh/bh_2 \
 	    -I$(srctree)/$(src)/$(css_folder)/$(kernels)/raw_aa_binning/raw_aa_binning_1.0 \
 	    -I$(srctree)/$(src)/$(css_folder)/isp/modes \
 	    -I$(srctree)/$(src)/$(css_folder)/$(runtime)/binary/interface \
 	    -I$(srctree)/$(src)/$(css_folder)/$(runtime)/bufq/interface \
 	    -I$(srctree)/$(src)/$(css_folder)/$(runtime)/debug/interface \
 	    -I$(srctree)/$(src)/$(css_folder)/$(runtime)/event/interface \
 	    -I$(srctree)/$(src)/$(css_folder)/$(runtime)/eventq/interface \
        -I$(srctree)/$(src)/$(css_folder)/$(runtime)/frame/interface \
        -I$(srctree)/$(src)/$(css_folder)/$(runtime)/ifmtr/interface \
        -I$(srctree)/$(src)/$(css_folder)/$(runtime)/inputfifo/interface \
        -I$(srctree)/$(src)/$(css_folder)/$(runtime)/isp_param/interface \
        -I$(srctree)/$(src)/$(css_folder)/$(runtime)/isys/interface \
        -I$(srctree)/$(src)/$(css_folder)/$(runtime)/pipeline/interface \
        -I$(srctree)/$(src)/$(css_folder)/$(runtime)/queue/interface \
        -I$(srctree)/$(src)/$(css_folder)/$(runtime)/rmgr/interface \
        -I$(srctree)/$(src)/$(css_folder)/$(runtime)/spctrl/interface \

DEFINES := -DHRT_HW -DHRT_ISP_CSS_CUSTOM_HOST -DHRT_USE_VIR_ADDRS -DHRT_KERNEL -D__HOST__
DEFINES += -DUSE_DYNAMIC_BIN
DEFINES += -DISP_POWER_GATING
DEFINES += -DUSE_INTERRUPTS
#DEFINES += -DUSE_SSSE3
DEFINES += -DPUNIT_CAMERA_BUSY
ifndef CONFIG_64BIT
DEFINES += -DUSE_KMEM_CACHE
endif
#Disable this define if using BYT D0
DEFINES += -DISOF_SILICON_BUG_WORKAROUND
DEFINES += -DCSS_2
DEFINES += -DSYSTEM_hive_isp_css_2400_system

EXTRA_CFLAGS := $(INCLUDES) $(DEFINES) -fno-common #-Werror

# sparse (0.4.4) doesn't understand _Bool at all.  This will eliminate lots of
# spurious "Initializer entry defined twice" errors for CSS code containing
# arrays of Boolean type.
CHECKFLAGS += -D_Bool=int
