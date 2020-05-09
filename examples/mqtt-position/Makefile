CONTIKI_PROJECT = mqtt-client-position
SMALL=1
all: $(CONTIKI_PROJECT)
MAKE_ROUTING=MAKE_ROUTING_RPL_LITE
CONTIKI = ../..

include $(CONTIKI)/Makefile.dir-variables
MODULES += $(CONTIKI_NG_APP_LAYER_DIR)/mqtt

-include $(CONTIKI)/Makefile.identify-target

MODULES_REL += arch/platform/$(TARGET)

PLATFORMS_ONLY = cc26x0-cc13x0 cc2538dk openmote zoul native simplelink z1 sky

include $(CONTIKI)/Makefile.include
