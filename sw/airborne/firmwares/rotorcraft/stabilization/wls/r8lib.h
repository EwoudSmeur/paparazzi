void gamma_values ( int *n_data, float *x, float *fx );
void gamma_log_values ( int *n_data, float *x, float *fx );
int i4_log_10 ( int i );
int i4_max ( int i1, int i2 );
int i4_min ( int i1, int i2 );
int i4_modp ( int i, int j );
int i4_power ( int i, int j );
int i4_sign ( int i );
int i4_uniform_ab ( int a, int b, int *seed );
int i4_wrap ( int ival, int ilo, int ihi );
float i4int_to_r8int ( int imin, int imax, int i, float rmin, float rmax );
void i4mat_print ( int m, int n, int a[], char *title );
void i4mat_print_some ( int m, int n, int a[], int ilo, int jlo, int ihi,
  int jhi, char *title );
void i4vec_copy ( int n, int a1[], int a2[] );
int *i4vec_indicator0_new ( int n );
int *i4vec_indicator1_new ( int n );
void i4vec_permute ( int n, int p[], int a[] );
void i4vec_print ( int n, int a[], char *title );
void i4vec_transpose_print ( int n, int a[], char *title );
void i4vec_zeros ( int n, int a[] );
int *i4vec_zeros_new ( int n );
float *legendre_zeros ( int order );
int perm0_check ( int n, int p[] );
int *perm0_uniform_new ( int n, int *seed );
int perm1_check ( int n, int p[] );
int *perm1_uniform_new ( int n, int *seed );
float r8_abs ( float x );
float r8_acos ( float c );
float r8_acosh ( float x );
float r8_add ( float x, float y );
float r8_agm ( float a, float b );
float r8_aint ( float x );
float r8_asin ( float c );
float r8_asinh ( float x );
float r8_atan ( float y, float x );
float r8_atanh ( float x );
float r8_big ( void );
float r8_cas ( float x );
float r8_ceiling ( float x );
float r8_choose ( int n, int k );
float r8_chop ( int place, float x );
float r8_cosd ( float degrees );
float r8_cot ( float angle );
float r8_cotd ( float degrees );
float r8_csc ( float theta );
float r8_cscd ( float degrees );
float r8_cube_root ( float x );
float r8_degrees ( float radians );
float r8_diff ( float x, float y, int n );
int r8_digit ( float x, int idigit );
float r8_divide_i4 ( int i, int j );
float r8_e ( void );
float r8_epsilon ( void );
float r8_epsilon_compute ( void );
float r8_exp ( float x );
float r8_factorial ( int n );
float r8_factorial_stirling ( int n );
void r8_factorial_values ( int *n_data, int *n, float *fn );
float r8_factorial2 ( int n );
void r8_factorial2_values ( int *n_data, int *n, float *f );
float r8_fall ( float x, int n );
void r8_fall_values ( int *n_data, float *x, int *n, float *f );
float r8_floor ( float x );
float r8_fraction ( int i, int j );
float r8_fractional ( float x );
float r8_gamma ( float x );
float r8_gamma_log ( float x );
float r8_huge ( void );
float r8_hypot ( float a, float b );
int r8_in_01 ( float a );
int r8_insignificant ( float r, float s );
int r8_is_inf ( float r );
int r8_is_int ( float r );
int r8_is_nan ( float r );
float r8_log_10 ( float x );
float r8_log_2 ( float x );
float r8_log_b ( float x, float b );
void r8_mant ( float x, int *s, float *r, int *l );
float r8_max ( float x, float y );
float r8_min ( float x, float y );
float r8_mod ( float x, float y );
float r8_modp ( float x, float y );
float r8_mop ( int i );
int r8_nint ( float x );
float r8_normal_01 ( int *seed );
float r8_normal_ab ( float a, float b, int *seed );
float r8_pi ( void );
float r8_pi_sqrt ( void );
float r8_power ( float r, int p );
float r8_power_fast ( float r, int p, int *mults );
void r8_print ( float r, char *title );
float r8_pythag ( float a, float b );
float r8_radians ( float degrees );
float r8_reverse_bytes ( float x );
float r8_rise ( float x, int n );
void r8_rise_values ( int *n_data, float *x, int *n, float *f );
float r8_round ( float x );
int r8_round_i4 ( float x );
float r8_round2 ( int nplace, float x );
float r8_roundb ( int base, int nplace, float x );
float r8_roundx ( int nplace, float x );
float r8_secd ( float degrees );
float r8_sech ( float x );
float r8_sign ( float x );
float r8_sign3 ( float x );
char r8_sign_char ( float x );
int r8_sign_match ( float r1, float r2 );
int r8_sign_match_strict ( float r1, float r2 );
int r8_sign_opposite ( float r1, float r2 );
int r8_sign_opposite_strict ( float r1, float r2 );
float r8_sign2 ( float x, float y );
void r8_sincos_sum ( float a, float b, float *d, float *e, float *f );
float r8_sind ( float degrees );
float r8_sqrt_i4 ( int i );
float r8_sum ( float x, float y );
void r8_swap ( float *x, float *y );
void r8_swap3 ( float *x, float *y, float *z );
float r8_tand ( float degrees );
float r8_tiny ( void );
void r8_to_dhms ( float r, int *d, int *h, int *m, int *s );
int r8_to_i4 ( float xmin, float xmax, float x, int ixmin, int ixmax );
float r8_to_r8_discrete ( float r, float rmin, float rmax, int nr );
float r8_uniform_ab ( float b, float c, int *seed );
float r8_uniform_01 ( int *seed );
void r8_unswap3 ( float *x, float *y, float *z );
float r8_walsh_1d ( float x, int digit );
float r8_wrap ( float r, float rlo, float rhi );
float r8_zeta ( float p );
float r82_dist_l2 ( float a1[2], float a2[2] );
void r82_print ( float a[2], char *title );
void r82_uniform_ab ( float b, float c, int *seed, float r[] );
void r82col_print_part ( int n, float a[], int max_print, char *title );
void r82poly2_print ( float a, float b, float c, float d, float e, float f );
int r82poly2_type ( float a, float b, float c, float d, float e, float f );
void r82poly2_type_print ( int type );
float *r82row_max ( int n, float a[] );
float *r82row_min ( int n, float a[] );
int r82row_order_type ( int n, float a[] );
void r82row_part_quick_a ( int n, float a[], int *l, int *r );
void r82row_permute ( int n, int p[], float a[] );
void r82row_print ( int n, float a[], char *title );
void r82row_print_part ( int n, float a[], int max_print, char *title );
int *r82row_sort_heap_index_a ( int n, float a[] );
void r82row_sort_quick_a ( int n, float a[] );
float r83_norm ( float x, float y, float z );
void r83col_print_part ( int n, float a[], int max_print, char *title );
float *r83row_max ( int n, float a[] );
float *r83row_min ( int n, float a[] );
void r83row_part_quick_a ( int n, float a[], int *l, int *r );
void r83row_print_part ( int n, float a[], int max_print, char *title );
void r83row_sort_quick_a ( int n, float a[] );
void r8block_delete ( int l, int m, int n, float ***a );
float *r8block_expand_linear ( int l, int m, int n, float x[], int lfat,
  int mfat, int nfat );
float ***r8block_new ( int l, int m, int n );
void r8block_print ( int l, int m, int n, float a[], char *title );
void r8cmat_delete ( int m, int n, float **a );
float **r8cmat_new ( int m, int n );
void r8cmat_print ( int m, int n, float **a, char *title );
void r8cmat_print_some ( int m, int n, float **a, int ilo, int jlo, int ihi,
  int jhi, char *title );
float *r8cmat_to_r8mat_new ( int m, int n, float **a );
float **r8cmat_zeros_new ( int m, int n );
float *r8block_zeros_new ( int l, int m, int n );
float r8int_to_r8int ( float rmin, float rmax, float r, float r2min,
  float r2max );
int r8int_to_i4int ( float rmin, float rmax, float r, int imin, int imax );
void r8mat_add ( int m, int n, float alpha, float a[], float beta, 
  float b[], float c[] );
float *r8mat_add_new ( int m, int n, float alpha, float a[], float beta, 
  float b[] );
float r8mat_amax ( int m, int n, float a[] );
float *r8mat_border_add ( int m, int n, float table[] );
float *r8mat_border_cut ( int m, int n, float table[] );
float *r8mat_cholesky_factor ( int n, float a[], int *flag );
float *r8mat_cholesky_factor_upper ( int n, float a[], int *flag );
void r8mat_cholesky_inverse ( int n, float a[] );
float *r8mat_cholesky_solve ( int n, float a[], float b[] );
float *r8mat_cholesky_solve_upper ( int n, float r[], float b[] );
void r8mat_copy ( int m, int n, float a1[], float a2[] );
float *r8mat_copy_new ( int m, int n, float a1[] );
float *r8mat_covariance ( int m, int n, float x[] );
float r8mat_det ( int n, float a[] );
float r8mat_det_2d ( float a[] );
float r8mat_det_3d ( float a[] );
float r8mat_det_4d ( float a[] );
float r8mat_det_5d ( float a[] );
void r8mat_diag_add_scalar ( int n, float a[], float s );
void r8mat_diag_add_vector ( int n, float a[], float v[] );
void r8mat_diag_get_vector ( int n, float a[], float v[] );
float *r8mat_diag_get_vector_new ( int n, float a[] );
void r8mat_diag_set_scalar ( int n, float a[], float s );
void r8mat_diag_set_vector ( int n, float a[], float v[] );
float *r8mat_diagonal_new ( int n, float diag[] );
float r8mat_diff_frobenius ( int m, int n, float a[], float b[] );
float *r8mat_expand_linear ( int m, int n, float x[], int mfat, int nfat );
float *r8mat_expand_linear2 ( int m, int n, float a[], int m2, int n2 );
float *r8mat_flip_cols_new ( int m, int n, float a[] );
float *r8mat_flip_rows_new ( int m, int n, float a[] );
void r8mat_fs ( int n, float a[], float x[] );
float *r8mat_fs_new ( int n, float a[], float b[] );
void r8mat_fss ( int n, float a[], int nb, float x[] );
float *r8mat_fss_new ( int n, float a[], int nb, float b[] );
float *r8mat_givens_post ( int n, float a[], int row, int col );
float *r8mat_givens_pre ( int n, float a[], int row, int col );
float *r8mat_hess ( float (*fx )( int n, float x[] ), int n, float x[] );
void r8mat_house_axh ( int n, float a[], float v[] );
float *r8mat_house_axh_new ( int n, float a[], float v[] );
float *r8mat_house_form ( int n, float v[] );
float *r8mat_house_hxa ( int n, float a[], float v[] );
float *r8mat_house_post ( int n, float a[], int row, int col );
float *r8mat_house_pre ( int n, float a[], int row, int col );
void r8mat_identity ( int n, float a[] );
float *r8mat_identity_new ( int n );
int r8mat_in_01 ( int m, int n, float a[] );
float *r8mat_indicator_new ( int m, int n );
int r8mat_insignificant ( int m, int n, float r[], float s[] );
float *r8mat_inverse_2d ( float a[] );
float *r8mat_inverse_3d ( float a[] );
float *r8mat_inverse_4d ( float a[] );
float r8mat_is_identity ( int n, float a[] );
float r8mat_is_symmetric ( int m, int n, float a[] );
float *r8mat_jac ( int m, int n, float eps,
  float *(*fx) ( int m, int n, float x[] ), float x[] );
float *r8mat_kronecker ( int m1, int n1, float a[], int m2, int n2,
  float b[] );
float *r8mat_l_inverse ( int n, float a[] );
void r8mat_l_print ( int m, int n, float a[], char *title );
float *r8mat_l_solve ( int n, float a[], float b[] );
float *r8mat_l1_inverse ( int n, float a[] );
float *r8mat_lt_solve ( int n, float a[], float b[] );
void r8mat_lu ( int m, int n, float a[], float l[], float p[], float u[] );
float r8mat_max ( int m, int n, float a[] );
void r8mat_max_index ( int m, int n, float a[], int *i_max, int *j_max );
float r8mat_maxcol_minrow ( int m, int n, float a[] );
float r8mat_maxrow_mincol ( int m, int n, float a[] );
float r8mat_mean ( int m, int n, float a[] );
float r8mat_min ( int m, int n, float a[] );
void r8mat_min_index ( int m, int n, float a[], int *i_min, int *j_min );
float r8mat_mincol_maxrow ( int m, int n, float a[] );
float r8mat_minrow_maxcol ( int m, int n, float a[] );
void r8mat_minvm ( int n1, int n2, float a[], float b[], float c[] );
float *r8mat_minvm_new ( int n1, int n2, float a[], float b[] );
void r8mat_mm ( int n1, int n2, int n3, float a[], float b[], float c[] );
float *r8mat_mm_new ( int n1, int n2, int n3, float a[], float b[] );
float *r8mat_mmt_new ( int n1, int n2, int n3, float a[], float b[] );
float *r8mat_mtm_new ( int n1, int n2, int n3, float a[], float b[] );
void r8mat_mtv ( int m, int n, float a[], float x[], float atx[] );
float *r8mat_mtv_new ( int m, int n, float a[], float x[] );
void r8mat_mv ( int m, int n, float a[], float x[], float ax[] );
float *r8mat_mv_new ( int m, int n, float a[], float x[] );
void r8mat_nint ( int m, int n, float a[] );
int r8mat_nonzeros ( int m, int n, float a[] );
float r8mat_norm_eis ( int m, int n, float a[] );
float r8mat_norm_fro ( int m, int n, float a[] );
float r8mat_norm_fro_affine ( int m, int n, float a1[], float a2[] );
float r8mat_norm_l1 ( int m, int n, float a[] );
float r8mat_norm_l2 ( int m, int n, float a[] );
float r8mat_norm_li ( int m, int n, float a[] );
float *r8mat_normal_01_new ( int m, int n, int *seed );
float *r8mat_nullspace ( int m, int n, float a[], int nullspace_size );
int r8mat_nullspace_size ( int m, int n, float a[] );
void r8mat_orth_uniform ( int n, int *seed, float a[] );
float *r8mat_orth_uniform_new ( int n, int *seed );
void r8mat_plot ( int m, int n, float a[], char *title );
char r8mat_plot_symbol ( float r );
float *r8mat_poly_char ( int n, float a[] );
float *r8mat_power ( int n, float a[], int npow );
void r8mat_power_method ( int n, float a[], float *r, float v[] );
void r8mat_print ( int m, int n, float a[], char *title );
void r8mat_print_some ( int m, int n, float a[], int ilo, int jlo, int ihi,
  int jhi, char *title );
void r8mat_ref ( int m, int n, float a[] );
float r8mat_rms ( int m, int n, float a[] );
void r8mat_row_copy ( int m, int n, int i, float v[], float a[] );
void r8mat_rref ( int m, int n, float a[] );
void r8mat_scale ( int m, int n, float s, float a[] );
int r8mat_significant ( int m, int n, float r[], float s[] );
int r8mat_solve ( int n, int rhs_num, float a[] );
float *r8mat_solve_2d ( float a[], float b[], float *det );
float *r8mat_solve_3d ( float a[], float b[], float *det );
float *r8mat_solve2 ( int n, float a[], float b[], int *ierror );
float r8mat_sum ( int m, int n, float a[] );
float *r8mat_symm_eigen ( int n, float x[], float q[] );
void r8mat_symm_jacobi ( int n, float a[] );
float **r8mat_to_r8cmat_new ( int m, int n, float a[] );
int r8mat_to_r8plu ( int n, float a[], int pivot[], float lu[] );
float **r8mat_to_r8rmat ( int m, int n, float a[] );
float r8mat_trace ( int n, float a[] );
void r8mat_transpose_in_place ( int n, float a[] );
float *r8mat_transpose_new ( int m, int n, float a[] );
void r8mat_transpose_print ( int m, int n, float a[], char *title );
void r8mat_transpose_print_some ( int m, int n, float a[], int ilo, int jlo,
  int ihi, int jhi, char *title );
float *r8mat_u_inverse ( int n, float a[] );
float *r8mat_u_solve ( int n, float a[], float b[] );
float *r8mat_u1_inverse ( int n, float a[] );
void r8mat_uniform_01 ( int m, int n, int *seed, float r[] );
float *r8mat_uniform_01_new ( int m, int n, int *seed );
void r8mat_uniform_ab ( int m, int n, float a, float b, int *seed, float r[] );
float *r8mat_uniform_ab_new ( int m, int n, float a, float b, int *seed );
void r8mat_uniform_abvec ( int m, int n, float a[], float b[], int *seed, float r[] );
float *r8mat_uniform_abvec_new ( int m, int n, float a[], float b[], int *seed );
float *r8mat_ut_solve ( int n, float a[], float b[] );
float *r8mat_vand2 ( int n, float x[] );
float r8mat_vtmv ( int m, int n, float x[], float a[], float y[] );
void r8mat_zeros ( int m, int n, float a[] );
float *r8mat_zeros_new ( int m, int n );
float r8plu_det ( int n, int pivot[], float lu[] );
void r8plu_inverse ( int n, int pivot[], float lu[], float a_inverse[] );
void r8plu_mul ( int n, int pivot[], float lu[], float x[], float b[] );
void r8plu_sol ( int n, int pivot[], float lu[], float b[], float x[] );
void r8plu_to_r8mat ( int n, int pivot[], float lu[], float a[] );
int r8poly_degree ( int na, float a[] );
float *r8poly_deriv ( int n, float c[], int p );
float r8poly_lagrange_0 ( int npol, float xpol[], float xval );
float r8poly_lagrange_1 ( int npol, float xpol[], float xval );
float r8poly_lagrange_2 ( int npol, float xpol[], float xval );
float *r8poly_lagrange_coef ( int npol, int ipol, float xpol[] );
void r8poly_lagrange_factor ( int npol, float xpol[], float xval,
  float *wval, float *dwdx );
int r8poly_lagrange_val ( int npol, int ipol, float xpol[], float xval,
  float *pval, float *dpdx );
int r8poly_order ( int na, float a[] );
void r8poly_print ( int n, float a[], char *title );
void r8poly_shift ( float scale, float shift, int n, float poly_cof[] );
float r8poly_value_horner ( int n, float a[], float x );
float *r8poly_values_horner ( int m, float c[], int n, float x[] );
float *r8poly_value_2d ( int m, float c[], int n, float x[], float y[] );
int r8poly2_ex ( float x1, float y1, float x2, float y2, float x3, float y3,
  float *x, float *y );
int r8poly2_ex2 ( float x1, float y1, float x2, float y2, float x3, float y3,
  float *x, float *y, float *a, float *b, float *c );
void r8poly2_rroot ( float a, float b, float c, float *r1, float *r2 );
void r8poly2_val ( float x1, float y1, float x2, float y2,
  float x3, float y3, float x, float *y, float *yp, float *ypp );
void r8poly2_val2 ( int ndata, float tdata[],
  float ydata[], int left, float tval, float *yval );
int r8r8_compare ( float x1, float y1, float x2, float y2 );
void r8r8_print ( float a1, float a2, char *title );
int r8r8r8_compare ( float x1, float y1, float z1, float x2, float y2,
  float z2 );
void r8r8r8vec_index_insert_unique ( int maxn, int *n, float x[], float y[],
  float z[], int indx[], float xval, float yval, float zval, int *ival,
  int *ierror );
void r8r8r8vec_index_search ( int n, float x[], float y[], float z[],
  int indx[], float xval, float yval, float zval, int *less, int *equal,
  int *more );
void r8r8vec_index_insert_unique ( int maxn, int *n, float x[], float y[],
  int indx[], float xval, float yval, int *ival, int *ierror );
void r8r8vec_index_search ( int n, float x[], float y[], int indx[],
  float xval, float yval, int *less, int *equal, int *more );
float **r8rmat_copy_new ( int m, int n, float **a );
void r8rmat_delete ( int m, int n, float **a );
float *r8rmat_fs_new ( int n, float **a, float b[] );
float **r8rmat_new ( int m, int n );
void r8rmat_print ( int m, int n, float **a, char *title );
void r8rmat_print_some ( int m, int n, float **a, int ilo, int jlo, int ihi,
  int jhi, char *title );
float *r8rmat_to_r8mat ( int m, int n, float **a );
float **r8rmat_zeros ( int m, int n );
void r8slmat_print ( int m, int n, float a[], char *title );
void r8vec_01_to_ab ( int n, float a[], float amax, float amin );
void r8vec_ab_to_01 ( int n, float a[] );
float *r8vec_ab_to_cd ( int n, float a[], float bmin, float bmax );
void r8vec_add ( int n, float a1[], float a2[] );
int r8vec_all_nonpositive ( int n, float a[] );
float r8vec_amax ( int n, float a[] );
int r8vec_amax_index ( int n, float a[] );
float r8vec_amin ( int n, float a[] );
int r8vec_amin_index ( int n, float a[] );
int r8vec_any_negative ( int n, float a[] );
int r8vec_any_nonzero ( int n, float a[] );
float *r8vec_any_normal ( int dim_num, float v1[] );
int r8vec_ascends ( int n, float x[] );
int r8vec_ascends_strictly ( int n, float x[] );
float r8vec_asum ( int n, float a[] );
void r8vec_bin ( int n, float x[], int bin_num, float bin_min, float bin_max,
  int bin[], float bin_limit[] );
void r8vec_bracket ( int n, float x[], float xval, int *left,
  int *right );
void r8vec_bracket2 ( int n, float x[], float xval, int start, int *left,
  int *right );
void r8vec_bracket3 ( int n, float t[], float tval, int *left );
void r8vec_bracket4 ( int nt, float t[], int ns, float s[], int left[] );
int r8vec_bracket5 ( int nd, float xd[], float xi );
int *r8vec_bracket6 ( int nd, float xd[], int ni, float xi[] );
float *r8vec_chebyspace_new ( int n, float a, float b );
float *r8vec_cheby1space_new ( int n, float a, float b );
float *r8vec_cheby2space_new ( int n, float a, float b );
float r8vec_circular_variance ( int n, float x[] );
int r8vec_compare ( int n, float a[], float b[] );
void r8vec_concatenate ( int n1, float a[], int n2, float b[], float c[] );
float *r8vec_concatenate_new ( int n1, float a[], int n2, float b[] );
float *r8vec_convolution ( int m, float x[], int n, float y[] );
float *r8vec_convolution_circ ( int n, float x[], float y[] );
void r8vec_copy ( int n, float a1[], float a2[] );
float *r8vec_copy_new ( int n, float a1[] );
float r8vec_correlation ( int n, float x[], float y[] );
float r8vec_covar ( int n, float x[], float y[] );
float r8vec_cross_product_2d ( float v1[2], float v2[2] );
float r8vec_cross_product_2d_affine ( float v0[2], float v1[2], float v2[2] );
float *r8vec_cross_product_3d ( float v1[3], float v2[3] );
float *r8vec_cross_product_3d_affine ( float v0[3], float v1[3], float v2[3] );
float *r8vec_cum_new ( int n, float a[] );
float *r8vec_cum0_new ( int n, float a[] );
float *r8vec_dif ( int n, float h );
float r8vec_diff_norm ( int n, float a[], float b[] );
float r8vec_diff_norm_l1 ( int n, float a[], float b[] );
float r8vec_diff_norm_l2 ( int n, float a[], float b[] );
float r8vec_diff_norm_li ( int n, float a[], float b[] );
float r8vec_diff_norm_squared ( int n, float a[], float b[] );
void r8vec_direct_product ( int factor_index, int factor_order,
  float factor_value[], int factor_num, int point_num, float x[] );
void r8vec_direct_product2 ( int factor_index, int factor_order,
  float factor_value[], int factor_num, int point_num, float w[] );
float r8vec_distance ( int dim_num, float v1[], float v2[] );
int r8vec_distinct ( int n, float x[] );
void r8vec_divide ( int n, float a[], float s );
float r8vec_dot_product ( int n, float a1[], float a2[] );
float r8vec_dot_product_affine ( int n, float v0[], float v1[], float v2[] );
float r8vec_entropy ( int n, float x[] );
int r8vec_eq ( int n, float a1[], float a2[] );
void r8vec_even ( int n, float alo, float ahi, float a[] );
float *r8vec_even_new ( int n, float alo, float ahi );
float r8vec_even_select ( int n, float xlo, float xhi, int ival );
void r8vec_even2 ( int maxval, int nfill[], int nold, float xold[],
  int *nval, float xval[] );
float r8vec_even2_select ( int n, float xlo, float xhi, int ival );
void r8vec_even3 ( int nold, int nval, float xold[], float xval[] );
float *r8vec_expand_linear ( int n, float x[], int fat );
float *r8vec_expand_linear2 ( int n, float x[], int before, int fat, 
  int after );
int *r8vec_first_index ( int n, float a[], float tol );
float r8vec_frac ( int n, float a[], int k );
float *r8vec_fraction ( int n, float x[] );
int r8vec_gt ( int n, float a1[], float a2[] );
void r8vec_heap_a ( int n, float a[] );
void r8vec_heap_d ( int n, float a[] );
int *r8vec_histogram ( int n, float a[], float a_lo, float a_hi, int histo_num );
float *r8vec_house_column ( int n, float a[], int k );
float r8vec_i4vec_dot_product ( int n, float r8vec[], int i4vec[] );
int r8vec_in_01 ( int n, float x[] );
int r8vec_in_ab ( int n, float x[], float a, float b );
void r8vec_index_delete_all ( int n, float x[], int indx[], float xval,
  int *n2, float x2[], int indx2[] );
void r8vec_index_delete_dupes ( int n, float x[], int indx[],
  int *n2, float x2[], int indx2[] );
void r8vec_index_delete_one ( int n, float x[], int indx[], float xval,
  int *n2, float x2[], int indx2[] );
void r8vec_index_insert ( int *n, float x[], int indx[], float xval );
void r8vec_index_insert_unique ( int *n, float x[], int indx[], float xval );
void r8vec_index_order ( int n, float x[], int indx[] );
void r8vec_index_search ( int n, float x[], int indx[], float xval, int *less,
  int *equal, int *more );
void r8vec_index_sort_unique ( int n, float x[], int *n2, float x2[],
  int indx2[] );
void r8vec_index_sorted_range ( int n, float r[], int indx[], float r_lo,
  float r_hi, int *i_lo, int *i_hi );
void r8vec_indexed_heap_d ( int n, float a[], int indx[] );
int r8vec_indexed_heap_d_extract ( int *n, float a[], int indx[] );
void r8vec_indexed_heap_d_insert ( int *n, float a[], int indx[],
  int indx_insert );
int r8vec_indexed_heap_d_max ( int n, float a[], int indx[] );
void r8vec_indicator0 ( int n, float a[] );
float *r8vec_indicator0_new ( int n );
void r8vec_indicator1 ( int n, float a[] );
float *r8vec_indicator1_new ( int n );
void r8vec_insert ( int n, float a[], int pos, float value );
int r8vec_insignificant ( int n, float r[], float s[] );
int r8vec_is_int ( int n, float a[] );
int r8vec_is_nonnegative ( int n, float x[] );
int r8vec_is_zero ( int n, float x[] );
float *r8vec_legendre_new ( int n, float a_first, float a_last );
void r8vec_linspace ( int n, float a_lo, float a_hi, float x[] );
float *r8vec_linspace_new ( int n, float a_lo, float a_hi );
float *r8vec_linspace2_new ( int n, float a_lo, float a_hi );
int r8vec_lt ( int n, float a1[], float a2[] );
void r8vec_mask_print ( int n, float a[], int mask_num, int mask[],
  char *title );
float r8vec_max ( int n, float dvec[] );
int r8vec_max_abs_index ( int n, float a[] );
int r8vec_max_index ( int n, float a[] );
float r8vec_mean ( int n, float x[] );
float r8vec_mean_geometric ( int n, float x[] );
float r8vec_median ( int n, float a[] );
void r8vec_mesh_2d ( int nx, int ny, float xvec[], float yvec[], 
  float xmat[], float ymat[] );
float *r8vec_midspace_new ( int n, float a_lo, float a_hi );
float r8vec_min ( int n, float dvec[] );
int r8vec_min_index ( int n, float a[] );
float r8vec_min_pos ( int n, float a[] );
int r8vec_mirror_next ( int n, float a[] );
int r8vec_negative_strict ( int n, float a[] );
void r8vec_nint ( int n, float a[] );
float *r8vec_nint_new ( int n, float a[] );
float r8vec_norm ( int n, float a[] );
float r8vec_norm_affine ( int n, float v0[], float v1[] );
float r8vec_norm_l0 ( int n, float a[] );
float r8vec_norm_l1 ( int n, float a[] );
float r8vec_norm_l2 ( int n, float a[] );
float r8vec_norm_li ( int n, float a[] );
float r8vec_norm_lp ( int n, float a[], float p );
void r8vec_normal_01 ( int n, int *seed, float x[] );
float *r8vec_normal_01_new ( int n, int *seed );
void r8vec_normal_ab ( int n, float b, float c, int *seed, float x[] );
float *r8vec_normal_ab_new ( int n, float b, float c, int *seed );
void r8vec_normalize ( int n, float a[] );
void r8vec_normalize_l1 ( int n, float a[] );
float r8vec_normsq ( int n, float a[] );
float r8vec_normsq_affine ( int n, float v0[], float v1[] );
float *r8vec_ones_new ( int n );
int r8vec_order_type ( int n, float x[] );
void r8vec_part_quick_a ( int n, float a[], int *l, int *r );
void r8vec_permute ( int n, int p[], float a[] );
void r8vec_permute_cyclic ( int n, int k, float a[] );
void r8vec_permute_uniform ( int n, float a[], int *seed );
void r8vec_polarize ( int n, float a[], float p[], float a_normal[],
  float a_parallel[] );
int r8vec_positive_strict ( int n, float a[] );
void r8vec_print ( int n, float a[], char *title );
void r8vec_print_16 ( int n, float a[], char *title );
void r8vec_print_part ( int n, float a[], int i_lo, int i_hi, char *title );
void r8vec_print_some ( int n, float a[], int max_print, char *title );
float r8vec_product ( int n, float a[] );
void r8vec_range ( int n, float x[], float xmin, float xmax, float y[],
  float *ymin, float *ymax );
void r8vec_range_2 ( int n, float a[], float *amin, float *amax );
void r8vec_reverse ( int n, float a[] );
float r8vec_rms ( int n, float a[] );
void r8vec_rotate ( int n, float a[], int m );
float *r8vec_running_average ( int n, float v[] );
float *r8vec_running_sign3 ( int n, float v[] );
float *r8vec_running_sum ( int n, float v[] );
float r8vec_scalar_triple_product ( float v1[3], float v2[3], float v3[3] );
void r8vec_scale ( float s, int n, float a[] );
int r8vec_search_binary_a ( int n, float a[], float aval );
void r8vec_shift ( int shift, int n, float x[] );
void r8vec_shift_circular ( int shift, int n, float x[] );
void r8vec_sort_bubble_a ( int n, float a[] );
void r8vec_sort_bubble_d ( int n, float a[] );
void r8vec_sort_heap_a ( int n, float a[] );
void r8vec_sort_heap_d ( int n, float a[] );
void r8vec_sort_heap_index_a ( int n, float a[], int indx[] );
int *r8vec_sort_heap_index_a_new ( int n, float a[] );
void r8vec_sort_heap_index_d ( int n, float a[], int indx[] );
int *r8vec_sort_heap_index_d_new ( int n, float a[] );
int *r8vec_sort_heap_mask_a ( int n, float a[], int mask_num, int mask[] );
void r8vec_sort_insert_a ( int n, float a[] );
int *r8vec_sort_insert_index_a ( int n, float a[] );
void r8vec_sort_quick_a ( int n, float a[] );
void r8vec_sort_shell_a ( int n, float a[] );
float *r8vec_sorted_merge_a ( int na, float a[], int nb, float b[], int *nc );
int r8vec_sorted_nearest ( int n, float a[], float value );
void r8vec_sorted_range ( int n, float r[], float r_lo, float r_hi,
  int *i_lo, int *i_hi );
void r8vec_sorted_split ( int n, float a[], float split, int *i_lt, int *i_gt );
void r8vec_sorted_undex ( int x_num, float x_val[], int x_unique_num,
  float tol, int undx[], int xdnu[] );
float *r8vec_sorted_unique ( int n, float a[], float tol, int *unique_num );
int r8vec_sorted_unique_count ( int n, float a[], float tol );
void r8vec_sorted_unique_hist ( int n, float a[], float tol, int maxuniq,
  int *unique_num, float auniq[], int acount[] );
int r8vec_split ( int n, float a[], float split );
float r8vec_std ( int n, float a[] );
void r8vec_step ( float x0, int n, float x[], float fx[] );
void r8vec_stutter ( int n, float a[], int m, float am[] );
float *r8vec_stutter_new ( int n, float a[], int m );
float r8vec_sum ( int n, float a[] );
void r8vec_swap ( int n, float a1[], float a2[] );
void r8vec_transpose_print ( int n, float a[], char *title );
void r8vec_undex ( int x_num, float x_val[], int x_unique_num, float tol,
  int undx[], int xdnu[] );
void r8vec_uniform_01 ( int n, int *seed, float r[] );
float *r8vec_uniform_01_new ( int n, int *seed );
void r8vec_uniform_ab ( int n, float a, float b, int *seed, float r[] );
float *r8vec_uniform_ab_new ( int n, float a, float b, int *seed );
void r8vec_uniform_abvec ( int n, float a[], float b[], int *seed, float r[] );
float *r8vec_uniform_abvec_new ( int n, float a[], float b[], int *seed );
float *r8vec_uniform_unit_new ( int m, int *seed );
int r8vec_unique_count ( int n, float a[], float tol );
int *r8vec_unique_index ( int n, float a[], float tol );
float r8vec_variance ( int n, float x[] );
float *r8vec_vector_triple_product ( float v1[3], float v2[3], float v3[3] );
void r8vec_write ( int n, float r[], char *output_file );
void r8vec_zeros ( int n, float x[] );
float *r8vec_zeros_new ( int n );
int r8vec2_compare ( int n, float a1[], float a2[], int i, int j );
void r8vec2_print ( int n, float a1[], float a2[], char *title );
void r8vec2_print_some ( int n, float x1[], float x2[], int max_print,
  char *title );
void r8vec2_sort_a ( int n, float a1[], float a2[] );
void r8vec2_sort_d ( int n, float a1[], float a2[] );
int *r8vec2_sort_heap_index_a ( int n, float x[], float y[] );
void r8vec2_sorted_unique ( int n, float a1[], float a2[], int *unique_num );
void r8vec2_sorted_unique_index ( int n, float a1[], float a2[],
  int *unique_num, int indx[] );
int r8vec2_sum_max_index ( int n, float a[], float b[] );
void r8vec3_print ( int n, float a1[], float a2[], float a3[], char *title );
float *roots_to_r8poly ( int n, float x[] );
int s_len_trim ( char *s );
void sort_heap_external ( int n, int *indx, int *i, int *j, int isgn );
void timestamp ( void );
