// кастомные malloc и free + проверка на двойной free, двинутые блоки и не из нащей

#include <stddef.h>
#define CLS_FREE 0
#define CLS_15  1
#define CLS_180 2


typedef union Header {
    struct {
        unsigned char cls;
    } h;
    void* _align;
} Header;


typedef struct FreeNode {
    struct FreeNode* next;
} FreeNode;

typedef struct Pool {
    FreeNode* free_list;
    size_t block_size;
} Pool;

#define ALIGN (sizeof(void*))

static size_t align_up(size_t x) {
    size_t a = ALIGN;
    return (x + (a - 1)) & ~(a - 1);
}

static Pool pool15;
static Pool pool180;
static int inited = 0;

static void init_pools() {
    if (inited) return;

    pool15.free_list = NULL;
    pool180.free_list = NULL;

    pool15.block_size  = align_up(sizeof(Header) + 15);
    pool180.block_size = align_up(sizeof(Header) + 180);

    inited = 1;
}

#define HP_SIZE (64u * 1024u)

static unsigned char heap[HP_SIZE];
static size_t heap_full = 0;

static void* arena_alloc(size_t bytes) {
    bytes = align_up(bytes);
    if (heap_full + bytes > HP_SIZE) return NULL;

    void* p = (void*)(heap + heap_full);
    heap_full += bytes;
    return p;
}

static int refill_pool(Pool* pool, unsigned char cls_id) {
    const size_t chunk = 1024u;
    size_t chunk_bytes = chunk;
    if (chunk_bytes < pool->block_size) chunk_bytes = pool->block_size;

    unsigned char* mem = (unsigned char*)arena_alloc(chunk_bytes);
    if (!mem) return 0;

    size_t nblocks = chunk_bytes / pool->block_size;
    if (nblocks == 0) return 0;

    for (size_t i = 0; i < nblocks; i++) {
        unsigned char* block = mem + i * pool->block_size;

        Header* h = (Header*)block;
        h->h.cls = CLS_FREE;

        void* user_ptr = (void*)(h + 1);

        FreeNode* node = (FreeNode*)user_ptr;
        node->next = pool->free_list;
        pool->free_list = node;
    }

    return 1;

}

void* my_malloc(size_t sz) {
    init_pools();

    Pool* pool = NULL;
    unsigned char cls = 0;

    if (sz == 15u) { pool = &pool15; cls = CLS_15; }
    else if (sz == 180u) { pool = &pool180; cls = CLS_180; }
    else return NULL;

    if (!pool->free_list) {
        if (!refill_pool(pool, cls)) return NULL;
    }

    FreeNode* node = pool->free_list;
    pool->free_list = node->next;

    Header* h = ((Header*)node) - 1;
    h->h.cls = cls;

    return (void*)node;
}

static int ptr_in_heap(void* p) {
    unsigned char* c = (unsigned char*)p;
    return (c >= heap) && (c < heap + HP_SIZE);
}

static int ptr_aligned(void* p) {
    return (((size_t)p) % sizeof(void*)) == 0;
}

static int ptr_matches_pool(void* p, Pool* pool) {
    unsigned char* block = (unsigned char*)p - sizeof(Header);
    size_t off = (size_t)(block - heap);
    return (off % pool->block_size) == 0;
}


void my_free(void* p) {
    if (!p) return;


    if (!ptr_in_heap(p)) return;
    if (!ptr_aligned(p)) return;


    Header* h = ((Header*)p) - 1;

    if (h->h.cls == CLS_FREE) return;

    Pool* pool = NULL;
    if (h->h.cls == CLS_15) pool = &pool15;
    else if (h->h.cls == CLS_180) pool = &pool180;
    else return;

    if (!ptr_matches_pool(p, pool)) return;

    h->h.cls = CLS_FREE;

    FreeNode* node = (FreeNode*)p;
    node->next = pool->free_list;
    pool->free_list = node;
}